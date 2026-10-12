// Aerodrone web UI: the downward camera with the tracked objects (tap one
// to follow it), the follow mode, Stop, the foam dart launcher (when the
// tracker reports one), the tracker's status, and the map with
// click-to-goal. Topics and commands: systems/aerodrone/tracking.md.
(function () {
  'use strict';
  const T = window.TrackingLogic;
  const $ = id => document.getElementById(id);
  const topicNames = T.topicNames(T.prefixFromQuery(window.location.search));
  const ACTIVE_STATES = new Set(['acquiring', 'tracking', 'lost']);
  const STATUS_STALE_S = 3;      // the tracker publishes its status at 2 Hz
  const FRAME_STALE_S = 2;       // images arrive at 2 Hz or more
  const PENDING_TIMEOUT_MS = 5000;
  const TAP_TOLERANCE_PX = 14;   // CSS pixels around a box that still select it
  const COLORS = {box: '#00e5ff', target: '#ffb300', label: 'rgba(0, 0, 0, 0.72)'};

  const state = {
    connected: false,
    frame: null,          // {image, stamp, receivedAt, width, height}
    frameTimes: [],
    cameraInfo: null,     // {width, height}
    history: new T.DetectionHistory(),
    drawn: null,          // the boxes on screen, for hit testing
    status: null,
    statusAt: 0,
    statusError: '',
    pending: null,        // {targetId, at}: tapped, not yet in the status
    lastCommand: null,    // the last command JSON published
    fix: null,
    odometry: null,
    gps: null,            // {status, at}: MAVROS' raw GPS fix
    correction: null,     // {source, at}: the navigation fusion's corrections
    fireSentAt: 0,        // a Fire press waits for the next status before another
  };
  // Read by the browser test (tools/tracking_ui_browser_check.py).
  window.__trackingUi = {state, topicNames};

  // ---------------------------------------------------------------- ROS
  const scheme = window.location.protocol === 'https:' ? 'wss' : 'ws';
  // rosbridge listens on 9092 (compositions/blueos_nav_core.yaml): on the
  // aircraft BlueOS's network manager holds the default 9090.
  const rosUrl = `${scheme}://${window.location.hostname || 'localhost'}:9092`;
  const ros = new ROSLIB.Ros();
  let reconnectTimer = null;

  function topic(name, messageType, options = {}) {
    return new ROSLIB.Topic(Object.assign({ros, name, messageType}, options));
  }

  const commandTopic = topic(topicNames.command, 'std_msgs/msg/String');
  const goalTopic = topic('/aerostack/map_goal', 'std_msgs/msg/String');
  // Advertised now (sent on connection, and again after a reconnection) so
  // the first command does not race the subscribers' discovery.
  commandTopic.advertise();
  goalTopic.advertise();

  topic(topicNames.image, 'sensor_msgs/msg/CompressedImage', {queue_length: 1})
    .subscribe(onImage);
  topic(topicNames.detections, 'vision_msgs/msg/Detection2DArray', {queue_length: 5})
    .subscribe(message => {
      state.history.add(message);
      scheduleRender();
    });
  topic(topicNames.cameraInfo, 'sensor_msgs/msg/CameraInfo',
        {throttle_rate: 1000, queue_length: 1})
    .subscribe(message => {
      if (message.width > 0 && message.height > 0) {
        const changed = !state.cameraInfo || state.cameraInfo.width !== message.width ||
          state.cameraInfo.height !== message.height;
        state.cameraInfo = {width: message.width, height: message.height};
        if (changed) layoutCamera();
      }
    });
  topic(topicNames.status, 'std_msgs/msg/String').subscribe(onStatus);
  topic('/navigation/fix', 'sensor_msgs/msg/NavSatFix', {throttle_rate: 200, queue_length: 1})
    .subscribe(onFix);
  topic('/navigation/odometry', 'nav_msgs/msg/Odometry', {throttle_rate: 200, queue_length: 1})
    .subscribe(message => {
      state.odometry = message;
      updateMapGoalReady();
      updateTargetMarker();
    });
  // The badge combines the GPS with the vision corrections
  // (TrackingLogic.navigationBadge): correction_source alone says
  // dead_reckoning whenever no DemNav/WildNav correction is active.
  topic('/mavros/global_position/raw/fix', 'sensor_msgs/msg/NavSatFix',
        {throttle_rate: 1000, queue_length: 1})
    .subscribe(message => {
      state.gps = {status: message.status.status, at: Date.now() / 1000};
      showNavigation();
    });
  topic('/navigation/correction_source', 'std_msgs/msg/String').subscribe(message => {
    state.correction = {source: message.data, at: Date.now() / 1000};
    showNavigation();
  });

  function showNavigation() {
    const badge = T.navigationBadge(state.gps, state.correction, Date.now() / 1000);
    $('nav-badge').textContent = badge.text;
    $('nav-badge').className = `badge ${badge.kind}`;
  }

  ros.on('connection', () => {
    state.connected = true;
    setBadge('ROSBridge connected', 'ok');
    updateButtons();
  });
  ros.on('error', () => setBadge('ROSBridge error', 'bad'));
  ros.on('close', () => {
    state.connected = false;
    setBadge('ROSBridge disconnected, retrying', 'bad');
    updateButtons();
    if (!reconnectTimer) {
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        ros.connect(rosUrl);
      }, 2000);
    }
  });
  ros.connect(rosUrl);

  function setBadge(text, kind) {
    const badge = $('ros-badge');
    badge.textContent = text;
    badge.title = rosUrl;
    badge.className = `badge ${kind || ''}`;
  }

  // roslib queues what is published while disconnected and sends it after
  // reconnecting: a stale command must not reach the drone, so refuse it.
  function publishCommand(result) {
    if (!state.connected) {
      flash('Not connected to ROSBridge: command not sent', true);
      return false;
    }
    commandTopic.publish(new ROSLIB.Message({data: result.json}));
    state.lastCommand = result.json;
    $('last-command').textContent = `Sent ${new Date().toLocaleTimeString()}: ${result.json}`;
    return true;
  }

  // ------------------------------------------------------------- camera
  let decoding = false;
  let queuedImage = null;

  function onImage(message) {
    queuedImage = message;   // keep only the newest while one is decoding
    if (!decoding) decodeNext();
  }

  function decodeNext() {
    const message = queuedImage;
    queuedImage = null;
    if (!message) return;
    decoding = true;
    const image = new Image();
    const done = () => {
      decoding = false;
      decodeNext();
    };
    image.onload = () => {
      const now = performance.now();
      const sizeChanged = !state.frame || state.frame.width !== image.naturalWidth ||
        state.frame.height !== image.naturalHeight;
      state.frame = {image, stamp: message.header.stamp, receivedAt: now,
                     width: image.naturalWidth, height: image.naturalHeight};
      state.frameTimes = state.frameTimes.filter(t => t > now - 5000).concat(now);
      if (sizeChanged && !state.cameraInfo) layoutCamera();
      scheduleRender();
      done();
    };
    image.onerror = done;
    const mime = /png/i.test(message.format || '') ? 'image/png' : 'image/jpeg';
    image.src = `data:${mime};base64,${message.data}`;
  }

  // The camera_info resolution the boxes are in (else the image's own).
  function imageSize() {
    if (state.cameraInfo) return state.cameraInfo;
    if (state.frame) return {width: state.frame.width, height: state.frame.height};
    return {width: 640, height: 480};
  }

  const canvas = $('camera');
  const wrap = $('camera-wrap');

  function layoutCamera() {
    const size = imageSize();
    wrap.style.setProperty('--camera-aspect', `${size.width} / ${size.height}`);
    const fit = T.fitRect(wrap.clientWidth, wrap.clientHeight, size.width, size.height);
    const ratio = window.devicePixelRatio || 1;
    canvas.style.width = `${fit.width}px`;
    canvas.style.height = `${fit.height}px`;
    canvas.width = Math.max(1, Math.round(fit.width * ratio));
    canvas.height = Math.max(1, Math.round(fit.height * ratio));
    scheduleRender();
  }
  new ResizeObserver(layoutCamera).observe(wrap);

  let renderQueued = false;
  function scheduleRender() {
    if (renderQueued) return;
    renderQueued = true;
    requestAnimationFrame(render);
  }

  function activeTargetId() {
    const status = state.status;
    if (status && ACTIVE_STATES.has(status.state) && status.target_id !== undefined &&
        status.target_id !== null && String(status.target_id) !== '') {
      return String(status.target_id);
    }
    return null;
  }

  function render() {
    renderQueued = false;
    const context = canvas.getContext('2d');
    const ratio = canvas.width / Math.max(canvas.clientWidth, 1);
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;
    const size = imageSize();
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.fillStyle = '#000';
    context.fillRect(0, 0, width, height);

    const frame = state.frame;
    const frameAge = frame ? (performance.now() - frame.receivedAt) / 1000 : Infinity;
    const drawn = {boxes: [], alpha: 0, exact: false, dt: null,
                   displayWidth: width, displayHeight: height,
                   imageWidth: size.width, imageHeight: size.height};
    let note = '';
    if (frame) {
      context.drawImage(frame.image, 0, 0, width, height);
      const match = state.history.match(frame.stamp);
      if (match) {
        drawn.boxes = match.boxes;
        drawn.exact = match.exact;
        drawn.dt = match.dt;
        drawn.alpha = T.boxAlpha(match.dt, frameAge, FRAME_STALE_S);
        drawBoxes(context, drawn);
      } else {
        note = 'No detections for this frame';
      }
      if (frameAge > FRAME_STALE_S) {
        context.fillStyle = 'rgba(0, 0, 0, 0.35)';
        context.fillRect(0, 0, width, height);
        note = `No new frame for ${frameAge.toFixed(0)} s`;
      }
    } else {
      note = `Waiting for the camera (${topicNames.image})`;
    }
    drawReticle(context, width, height);
    state.drawn = drawn;
    $('camera-note').textContent = note;
    const fps = state.frameTimes.length > 1 ? (state.frameTimes.length - 1) /
      ((state.frameTimes[state.frameTimes.length - 1] - state.frameTimes[0]) / 1000) : 0;
    $('frame-info').textContent = frame
      ? `${size.width}x${size.height}, ${fps.toFixed(1)} fps, ${drawn.boxes.length} objects`
      : '';
  }

  function drawBoxes(context, drawn) {
    const target = activeTargetId();
    const pending = state.pending && state.pending.targetId;
    const sx = drawn.displayWidth / drawn.imageWidth;
    const sy = drawn.displayHeight / drawn.imageHeight;
    context.save();
    context.globalAlpha = drawn.alpha;
    context.font = '600 13px system-ui, sans-serif';
    context.textBaseline = 'top';
    // The target last, on top of the others.
    const ordered = drawn.boxes.slice().sort(
      (a, b) => (a.id === target || a.id === pending) - (b.id === target || b.id === pending));
    for (const box of ordered) {
      const isTarget = box.id !== '' && box.id === target;
      const isPending = !isTarget && box.id !== '' && box.id === pending;
      const left = (box.cx - box.width / 2) * sx;
      const top = (box.cy - box.height / 2) * sy;
      const boxWidth = box.width * sx;
      const boxHeight = box.height * sy;
      context.strokeStyle = isTarget || isPending ? COLORS.target : COLORS.box;
      context.lineWidth = isTarget ? 4 : isPending ? 3 : 2;
      context.setLineDash(isPending ? [8, 5] : []);
      context.strokeRect(left, top, boxWidth, boxHeight);
      context.setLineDash([]);
      const score = box.score === null ? '' : ` ${Math.round(box.score * 100)}%`;
      const label = `${isTarget ? 'TARGET ' : ''}#${box.id || '?'} ${box.className}${score}`;
      const labelWidth = context.measureText(label).width + 8;
      const labelLeft = Math.max(0, Math.min(left, drawn.displayWidth - labelWidth));
      const labelTop = top >= 18 ? top - 18 : top + boxHeight;
      context.fillStyle = COLORS.label;
      context.fillRect(labelLeft, labelTop, labelWidth, 18);
      context.fillStyle = isTarget || isPending ? COLORS.target : COLORS.box;
      context.fillText(label, labelLeft + 4, labelTop + 2);
    }
    context.restore();
  }

  // The image centre: where centered mode keeps the target.
  function drawReticle(context, width, height) {
    const x = width / 2;
    const y = height / 2;
    context.save();
    context.strokeStyle = 'rgba(255, 255, 255, 0.55)';
    context.lineWidth = 1;
    context.beginPath();
    context.moveTo(x - 10, y);
    context.lineTo(x + 10, y);
    context.moveTo(x, y - 10);
    context.lineTo(x, y + 10);
    context.stroke();
    context.restore();
  }

  canvas.addEventListener('click', event => {
    const drawn = state.drawn;
    if (!drawn || !drawn.boxes.length) {
      flash('No objects in this frame', true);
      return;
    }
    // Do not pick a target from an old picture.
    if ((performance.now() - state.frame.receivedAt) / 1000 > FRAME_STALE_S) {
      flash('The camera view is old: wait for a new frame', true);
      return;
    }
    const rect = canvas.getBoundingClientRect();
    const point = T.displayToImage(event.clientX - rect.left, event.clientY - rect.top,
                                   rect.width, rect.height, drawn.imageWidth, drawn.imageHeight);
    const tolerance = TAP_TOLERANCE_PX * drawn.imageWidth / rect.width;
    const box = T.hitTest(drawn.boxes, point.x, point.y, tolerance);
    if (!box) {
      flash('No object there: tap inside a box', true);
      return;
    }
    follow(box.id, box.className);
  });

  function follow(targetId, className) {
    const result = T.trackCommand(targetId, currentMode(), modeInputs());
    if (!result.ok) {
      flash(result.error, true);
      $('mode-error').textContent = result.error;
      return;
    }
    if (publishCommand(result)) {
      state.pending = {targetId: String(targetId), at: Date.now()};
      flash(`Following #${targetId}${className ? ` (${className})` : ''}, ` +
            `${currentMode() === 'slant' ? 'slant range' : 'centered'}`);
      scheduleRender();
      updateButtons();
    }
  }

  let flashTimer = null;
  function flash(text, isError) {
    const hint = $('hint');
    hint.textContent = text;
    hint.className = isError ? 'error' : '';
    clearTimeout(flashTimer);
    flashTimer = setTimeout(() => {
      hint.textContent = 'Tap an object to follow it';
      hint.className = '';
    }, 4000);
  }

  // ------------------------------------------------------------ controls
  const modeInputsEls = Array.from(document.querySelectorAll('input[name="mode"]'));

  function currentMode() {
    const checked = modeInputsEls.find(input => input.checked);
    return checked ? checked.value : 'centered';
  }

  function modeInputs() {
    return {altitude: $('altitude-agl').value, slantRange: $('slant-range').value,
            depression: $('depression').value};
  }

  function refreshMode() {
    const mode = currentMode();
    for (const input of modeInputsEls) {
      input.parentElement.classList.toggle('checked', input.checked);
    }
    $('centered-fields').hidden = mode !== 'centered';
    $('slant-fields').hidden = mode !== 'slant';
    const result = T.validateMode(mode, modeInputs());
    $('mode-error').textContent = result.ok ? '' : result.error;
    for (const id of ['altitude-agl', 'slant-range', 'depression']) {
      $(id).classList.toggle('invalid', !result.ok && !$(id).closest('[hidden]'));
    }
    const depression = T.parseNumber($('depression').value);
    const range = T.parseNumber($('slant-range').value);
    let derived = '';
    if (Number.isFinite(depression) && depression >= T.LIMITS.minDepressionDeg &&
        depression <= T.LIMITS.maxDepressionDeg) {
      const bounds = T.slantRangeBounds(depression);
      derived = `Range ${bounds.min.toFixed(1)}-${bounds.max.toFixed(1)} m at this angle.`;
      if (Number.isFinite(range) && range > 0) {
        const geometry = T.slantGeometry(range, depression);
        derived = `Stand-off ${geometry.standoffM.toFixed(1)} m, ` +
          `${geometry.heightM.toFixed(1)} m above the target. ${derived}`;
      }
    }
    $('slant-derived').textContent = derived;
    updateButtons();
  }

  for (const input of modeInputsEls) {
    input.addEventListener('change', refreshMode);
    input.addEventListener('focus', () => input.parentElement.classList.add('focused'));
    input.addEventListener('blur', () => input.parentElement.classList.remove('focused'));
  }
  for (const id of ['altitude-agl', 'slant-range', 'depression']) {
    $(id).addEventListener('input', refreshMode);
  }

  function currentTargetId() {
    return (state.pending && state.pending.targetId) || activeTargetId();
  }

  function updateButtons() {
    showLauncher();
    $('stop').disabled = !state.connected;
    const target = currentTargetId();
    const valid = T.validateMode(currentMode(), modeInputs()).ok;
    const apply = $('apply');
    apply.disabled = !(state.connected && target && valid);
    apply.textContent = target ? `Apply to target #${target}` : 'Apply to the current target';
    updateMapGoalReady();
  }

  $('apply').addEventListener('click', () => {
    const target = currentTargetId();
    if (target) follow(target, '');
  });

  $('stop').addEventListener('click', () => {
    if (publishCommand(T.stopCommand())) {
      state.pending = null;
      flash('Stop sent: hovering');
      scheduleRender();
      updateButtons();
    }
  });

  // ------------------------------------------------------------ launcher
  // tracking.md, "Foam dart launcher". The tracker gates every shot; Fire is
  // enabled only while its fresh status says every gate holds for the
  // target it tracks (TrackingLogic.launcherView), and one press sends one
  // command naming that target.
  function statusAge() {
    return state.statusAt ? (Date.now() - state.statusAt) / 1000 : Infinity;
  }

  function showLauncher() {
    const view = T.launcherView(state.status, statusAge(), state.connected);
    $('launcher-card').hidden = !view.visible;
    if (!view.visible) return;
    const chip = $('launcher-chip');
    chip.textContent = view.state;
    chip.className = `chip ${/^[a-z]+$/.test(view.state) ? view.state : ''}`;
    const armed = $('launcher-armed');
    armed.textContent = view.armed ? `Armed${view.ready ? '' : ' (not ready)'}` : 'Disarmed';
    armed.className = `badge ${view.armed ? (view.ready ? 'warn' : 'bad') : 'ok'}`;
    $('launcher-darts').textContent =
      `Darts ${view.darts === null ? '-' : view.darts}, shots ${view.shots === null ? '-' : view.shots}`;
    $('launcher-arm').disabled = !view.arm;
    $('launcher-disarm').disabled = !view.disarm;
    const waiting = state.fireSentAt > 0 && state.statusAt <= state.fireSentAt &&
      Date.now() - state.fireSentAt < 3000;
    const fire = $('fire');
    fire.disabled = !view.fire.enabled || waiting;
    fire.textContent = view.fire.enabled
      ? `Fire at #${state.status.target_id} (${state.status.target_class || 'target'})` : 'Fire';
    const block = $('fire-block');
    block.textContent = view.fire.enabled
      ? (waiting ? 'Fire sent: waiting for the tracker' : 'Every check holds: one press, one dart')
      : `Fire blocked: ${view.fire.reason}`;
    block.className = `fire-block${view.fire.enabled ? ' clear' : ''}`;
    $('reload').disabled = !view.reload || !T.reloadCommand($('reload-darts').value).ok;
    $('launcher-last').textContent = view.lastFire ? `Last fire: ${view.lastFire}` : '';
    const list = $('fire-checks');
    const text = view.checks.map(check => `${check.name}:${check.ok}`).join(',');
    if (list.dataset.checks !== text) {
      list.dataset.checks = text;
      list.replaceChildren(...view.checks.map(check => {
        const item = document.createElement('li');
        item.textContent = `${check.ok ? '\u2713' : '\u2717'} ${check.label}`;
        item.title = check.name;
        if (!check.ok) item.className = 'no';
        return item;
      }));
    }
  }

  $('fire').addEventListener('click', () => {
    const view = T.launcherView(state.status, statusAge(), state.connected);
    if (!view.fire.enabled) {
      flash(`Fire blocked: ${view.fire.reason}`, true);
      return;
    }
    const result = T.fireCommand(state.status.target_id);
    if (!result.ok) {
      flash(result.error, true);
      return;
    }
    if (publishCommand(result)) {
      state.fireSentAt = Date.now();
      flash(`Fire sent at #${state.status.target_id}`);
      showLauncher();
    }
  });

  for (const [id, armed] of [['launcher-arm', true], ['launcher-disarm', false]]) {
    $(id).addEventListener('click', () => {
      if (publishCommand(T.launcherArmCommand(armed))) {
        flash(armed ? 'Launcher arm sent' : 'Launcher disarm sent');
      }
    });
  }

  $('reload').addEventListener('click', () => {
    const result = T.reloadCommand($('reload-darts').value);
    if (!result.ok) {
      flash(result.error, true);
      return;
    }
    if (publishCommand(result)) flash(`Reload sent: ${result.command.darts} darts`);
  });
  $('reload-darts').addEventListener('input', showLauncher);

  // -------------------------------------------------------------- status
  function onStatus(message) {
    const parsed = T.parseStatus(message.data);
    state.statusAt = Date.now();
    if (parsed.error) {
      state.statusError = parsed.error;
    } else {
      state.status = parsed.status;
      state.statusError = '';
      if (state.pending && activeTargetId() === state.pending.targetId) state.pending = null;
    }
    showStatus();
    updateTargetMarker();
    updateButtons();
    scheduleRender();
  }

  function number(value, digits) {
    return Number.isFinite(value) ? value.toFixed(digits) : '-';
  }

  function showStatus() {
    const status = state.status;
    const age = state.statusAt ? (Date.now() - state.statusAt) / 1000 : Infinity;
    const stale = age > STATUS_STALE_S;
    const chip = $('state-chip');
    const stateName = status && typeof status.state === 'string' ? status.state : '';
    chip.textContent = stateName || 'no status';
    chip.className = `chip ${/^[a-z]+$/.test(stateName) ? stateName : ''} ${stale ? 'stale' : ''}`;
    const target = activeTargetId();
    const line = $('state-line');
    while (line.childNodes.length > 1) line.removeChild(line.lastChild);
    let summary = target ? `#${target}` : '';
    if (state.pending && state.pending.targetId !== target) {
      summary += ` (requested #${state.pending.targetId})`;
    }
    if (stale && state.statusAt) summary += ` (status ${age.toFixed(0)} s old)`;
    line.appendChild(document.createTextNode(summary));

    if (!status) {
      $('st-state').textContent = state.statusError || `waiting for ${topicNames.status}`;
      return;
    }
    $('st-state').textContent = `${stateName || '-'}${stale ? ' (stale)' : ''}` +
      (state.statusError ? ` (last message bad: ${state.statusError})` : '');
    $('st-mode').textContent = status.mode || '-';
    let targetText = status.target_id === undefined || status.target_id === null ||
      status.target_id === '' ? '-' : `#${status.target_id}`;
    const box = target && state.drawn &&
      state.drawn.boxes.find(item => item.id === target);
    if (target) targetText += box ? ` ${box.className}, in view` : ', not in view';
    $('st-target').textContent = targetText;
    $('st-range').textContent = Number.isFinite(status.slant_range_m) ||
      Number.isFinite(status.depression_deg)
      ? `${number(status.slant_range_m, 1)} m at ${number(status.depression_deg, 0)} deg`
      : '-';
    const position = status.target;
    $('st-position').textContent = position && typeof position === 'object'
      ? `E ${number(position.east, 1)}, N ${number(position.north, 1)}, ` +
        `U ${number(position.up, 1)} m; ${number(position.speed_mps, 1)} m/s`
      : '-';
    $('st-message').textContent = status.message || '-';
    $('st-age').textContent = `${age.toFixed(1)} s ago`;
  }

  setInterval(() => {
    if (state.pending && Date.now() - state.pending.at > PENDING_TIMEOUT_MS) {
      flash(`The tracker has not taken #${state.pending.targetId}`, true);
      state.pending = null;
      updateButtons();
    }
    showStatus();
    showLauncher();
    showNavigation();
    scheduleRender();
  }, 500);

  // ----------------------------------------------------------------- map
  const map = L.map('map').setView([51.505, -0.09], 16);
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors'
  }).addTo(map);
  new ResizeObserver(() => map.invalidateSize()).observe($('map'));

  let selected = null;
  let goalMarker = null;
  let droneMarker = null;
  let targetMarker = null;
  let centered = false;

  function onFix(message) {
    if (!Number.isFinite(message.latitude) || !Number.isFinite(message.longitude)) return;
    state.fix = message;
    const position = [message.latitude, message.longitude];
    if (!droneMarker) {
      droneMarker = L.circleMarker(position, {radius: 7, color: '#ffffff', weight: 2,
                                              fillColor: '#4cc3ff', fillOpacity: 0.9})
        .bindTooltip('Drone').addTo(map);
    } else {
      droneMarker.setLatLng(position);
    }
    if (!centered) {
      map.setView(position, 17);
      centered = true;
    }
    updateMapGoalReady();
    updateTargetMarker();
  }

  // The target's ENU position on the map, converted like the map goal.
  function updateTargetMarker() {
    const status = state.status;
    const target = status && status.target;
    const ready = target && ACTIVE_STATES.has(status.state) && state.fix && state.odometry &&
      Number.isFinite(target.east) && Number.isFinite(target.north);
    if (!ready) {
      if (targetMarker) {
        targetMarker.remove();
        targetMarker = null;
      }
      return;
    }
    const point = T.enuToLatLon(target.east, target.north, state.fix,
                                state.odometry.pose.pose.position);
    const label = `Target #${status.target_id} (${status.state})`;
    if (!targetMarker) {
      targetMarker = L.circleMarker([point.lat, point.lon], {
        radius: 9, color: '#000000', weight: 2, fillColor: COLORS.target, fillOpacity: 0.95,
      }).bindTooltip(label).addTo(map);
    } else {
      targetMarker.setLatLng([point.lat, point.lon]);
      targetMarker.setTooltipContent(label);
    }
  }

  function updateMapGoalReady() {
    $('send').disabled = !(state.connected && selected && state.fix && state.odometry);
  }

  map.on('click', event => {
    selected = event.latlng;
    if (goalMarker) {
      goalMarker.setLatLng(selected);
    } else {
      goalMarker = L.marker(selected).addTo(map);
    }
    $('selection').value = `Goal ${selected.lat.toFixed(7)}, ${selected.lng.toFixed(7)}`;
    updateMapGoalReady();
  });

  $('send').addEventListener('click', () => {
    if (!selected || !state.fix || !state.odometry) return;
    const z = T.parseNumber($('altitude').value);
    if (!Number.isFinite(z)) {
      $('selection').value = 'Goal altitude must be a number';
      return;
    }
    if (!state.connected) {
      $('selection').value = 'Not connected to ROSBridge: goal not sent';
      return;
    }
    const goal = T.latLonToEnu(selected.lat, selected.lng, state.fix,
                               state.odometry.pose.pose.position);
    goalTopic.publish(new ROSLIB.Message({data: `${goal.east};${goal.north};${z}`}));
    $('selection').value =
      `Goal sent: ${goal.east.toFixed(1)} E, ${goal.north.toFixed(1)} N, ${z.toFixed(1)} m`;
  });

  refreshMode();
  layoutCamera();
}());
