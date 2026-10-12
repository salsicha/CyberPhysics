// Pure logic of the aerodrone web UI (index.html): tracking command
// validation and JSON, detection/frame time alignment, display <-> image
// pixel mapping, box hit testing, the map's ENU conversion and the
// navigation badge. No DOM and
// no ROS here, so tests/test_aerodrone_tracking_ui.py runs it under node.
// The command and topic contract is systems/aerodrone/tracking.md.
(function (root, factory) {
  const api = factory();
  if (typeof module === 'object' && module.exports) {
    module.exports = api;
  } else {
    root.TrackingLogic = api;
  }
}(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // Tracker limits from systems/aerodrone/tracking.md ("Limits").
  const LIMITS = Object.freeze({
    minAglM: 15,
    maxAglM: 120,
    minDepressionDeg: 60,
    maxDepressionDeg: 90,
  });
  // Detections drawn on a frame whose stamp is at most this far from theirs.
  const MAX_MATCH_DT_S = 0.5;
  const METRES_PER_DEGREE = 111320.0;
  const DEFAULT_PREFIX = '/tracking';

  function topicNames(prefix) {
    const base = prefix || DEFAULT_PREFIX;
    return {
      detections: `${base}/detections`,
      image: `${base}/image/compressed`,
      cameraInfo: `${base}/camera_info`,
      status: `${base}/status`,
      command: `${base}/command`,
    };
  }

  // A ROS topic prefix from the page's ?prefix= query, or the default.
  function prefixFromQuery(search) {
    const value = new URLSearchParams(search || '').get('prefix');
    if (value && /^(\/[A-Za-z0-9_]+)+$/.test(value)) return value;
    return DEFAULT_PREFIX;
  }

  // A number typed in an input: null when empty, NaN when not a finite number.
  function parseNumber(text) {
    const trimmed = String(text === undefined || text === null ? '' : text).trim();
    if (trimmed === '') return null;
    const value = Number(trimmed);
    return Number.isFinite(value) ? value : NaN;
  }

  function fmt(value) {
    return Number.isInteger(value) ? String(value) : value.toFixed(1);
  }

  // Centered mode: an empty height keeps the current height above ground
  // (the tracker's default, clamped to the limits), so the field is omitted.
  function validateCentered(altitudeText) {
    const altitude = parseNumber(altitudeText);
    if (altitude === null) return {ok: true, params: {}};
    if (Number.isNaN(altitude)) {
      return {ok: false, error: 'Height above ground must be a number'};
    }
    if (altitude < LIMITS.minAglM || altitude > LIMITS.maxAglM) {
      return {ok: false, error:
        `Height above ground must be ${LIMITS.minAglM}-${LIMITS.maxAglM} m`};
    }
    return {ok: true, params: {altitude_agl_m: altitude}};
  }

  // The slant ranges whose height above the target, R sin(theta), stays
  // within the height limits at this depression angle.
  function slantRangeBounds(depressionDeg) {
    const sine = Math.sin(depressionDeg * Math.PI / 180);
    return {min: LIMITS.minAglM / sine, max: LIMITS.maxAglM / sine};
  }

  function slantGeometry(slantRangeM, depressionDeg) {
    const angle = depressionDeg * Math.PI / 180;
    return {
      standoffM: slantRangeM * Math.cos(angle),
      heightM: slantRangeM * Math.sin(angle),
    };
  }

  function validateSlant(rangeText, depressionText) {
    const range = parseNumber(rangeText);
    const depression = parseNumber(depressionText);
    if (depression === null || Number.isNaN(depression)) {
      return {ok: false, error: 'Depression angle must be a number'};
    }
    if (depression < LIMITS.minDepressionDeg || depression > LIMITS.maxDepressionDeg) {
      return {ok: false, error: `Depression angle must be ` +
        `${LIMITS.minDepressionDeg}-${LIMITS.maxDepressionDeg} deg`};
    }
    if (range === null || Number.isNaN(range)) {
      return {ok: false, error: 'Slant range must be a number'};
    }
    const bounds = slantRangeBounds(depression);
    // A small tolerance so the displayed, rounded bounds are accepted.
    if (range < bounds.min - 0.05 || range > bounds.max + 0.05) {
      return {ok: false, error: `Slant range must be ${fmt(bounds.min)}-` +
        `${fmt(bounds.max)} m at ${fmt(depression)} deg (height above the ` +
        `target ${LIMITS.minAglM}-${LIMITS.maxAglM} m)`};
    }
    return {ok: true, params: {slant_range_m: range, depression_deg: depression}};
  }

  function validateMode(mode, inputs) {
    if (mode === 'centered') return validateCentered(inputs.altitude);
    if (mode === 'slant') return validateSlant(inputs.slantRange, inputs.depression);
    return {ok: false, error: `Unknown mode ${mode}`};
  }

  // {"action": "track", "target_id": "7", "mode": ..., <mode params>}
  function trackCommand(targetId, mode, inputs) {
    if (targetId === undefined || targetId === null || String(targetId) === '') {
      return {ok: false, error: 'The object has no track ID'};
    }
    const checked = validateMode(mode, inputs);
    if (!checked.ok) return checked;
    const command = Object.assign(
      {action: 'track', target_id: String(targetId), mode}, checked.params);
    return {ok: true, command, json: JSON.stringify(command)};
  }

  function stopCommand() {
    const command = {action: 'stop'};
    return {ok: true, command, json: JSON.stringify(command)};
  }

  function stampSeconds(stamp) {
    return stamp ? stamp.sec + stamp.nanosec * 1e-9 : NaN;
  }

  // a - b in seconds, exact to the nanosecond for nearby stamps.
  function stampDifference(a, b) {
    return (a.sec - b.sec) + (a.nanosec - b.nanosec) * 1e-9;
  }

  function stampKey(stamp) {
    return stamp ? `${stamp.sec}.${stamp.nanosec}` : '';
  }

  // vision_msgs/Detection2DArray (as rosbridge sends it) to plain boxes in
  // the pixels of /tracking/camera_info's image.
  function parseDetections(message) {
    const boxes = [];
    for (const detection of (message && message.detections) || []) {
      const bbox = detection.bbox || {};
      const center = (bbox.center && (bbox.center.position || bbox.center)) || {};
      const result = (detection.results && detection.results[0]) || null;
      const hypothesis = result ? (result.hypothesis || result) : {};
      const box = {
        id: detection.id === undefined || detection.id === null ? '' : String(detection.id),
        className: hypothesis.class_id ? String(hypothesis.class_id) : '',
        score: Number.isFinite(hypothesis.score) ? hypothesis.score : null,
        cx: Number(center.x),
        cy: Number(center.y),
        width: Number(bbox.size_x),
        height: Number(bbox.size_y),
      };
      if ([box.cx, box.cy, box.width, box.height].every(Number.isFinite) &&
          box.width >= 0 && box.height >= 0) {
        boxes.push(box);
      }
    }
    return boxes;
  }

  // Recent detection messages by stamp, for matching them to a frame.
  class DetectionHistory {
    constructor(keepSeconds = 5, maxEntries = 100) {
      this.keepSeconds = keepSeconds;
      this.maxEntries = maxEntries;
      this.entries = [];
    }

    add(message) {
      const stamp = message.header && message.header.stamp;
      const entry = {key: stampKey(stamp), stamp, seconds: stampSeconds(stamp),
                     boxes: parseDetections(message)};
      if (!Number.isFinite(entry.seconds)) return;
      // A stamp behind the history (a simulator restart, a replay) clears it.
      const newest = this.entries[this.entries.length - 1];
      if (newest && entry.seconds < newest.seconds - this.keepSeconds) this.entries = [];
      this.entries = this.entries.filter(item => item.key !== entry.key);
      this.entries.push(entry);
      this.entries.sort((a, b) => a.seconds - b.seconds);
      const latest = this.entries[this.entries.length - 1].seconds;
      this.entries = this.entries.filter(
        item => item.seconds >= latest - this.keepSeconds).slice(-this.maxEntries);
    }

    // The detections with the frame's stamp, else the nearest within
    // maxDt (the later one on a tie), else null.
    match(frameStamp, maxDt = MAX_MATCH_DT_S) {
      const key = stampKey(frameStamp);
      let best = null;
      for (const entry of this.entries) {
        if (entry.key === key) return {boxes: entry.boxes, dt: 0, exact: true};
        const dt = Math.abs(stampDifference(entry.stamp, frameStamp));
        if (dt <= maxDt && (!best || dt <= best.dt)) {
          best = {boxes: entry.boxes, dt, exact: false};
        }
      }
      return best;
    }
  }

  // Opacity of a frame's boxes: full for its own detections, fading to 0.35
  // for detections MAX_MATCH_DT_S away, halved while the frame itself is
  // old (no new image for staleAfterS of wall time).
  function boxAlpha(dt, frameAgeS, staleAfterS = 2) {
    let alpha = dt <= 0 ? 1 : Math.max(0.35, 1 - 0.65 * Math.min(dt / MAX_MATCH_DT_S, 1));
    if (frameAgeS > staleAfterS) alpha *= 0.5;
    return alpha;
  }

  // The largest width x height with the image's aspect that fits the area.
  function fitRect(availableWidth, availableHeight, imageWidth, imageHeight) {
    if (!(availableWidth > 0 && availableHeight > 0 && imageWidth > 0 && imageHeight > 0)) {
      return {width: 0, height: 0};
    }
    const scale = Math.min(availableWidth / imageWidth, availableHeight / imageHeight);
    return {width: imageWidth * scale, height: imageHeight * scale};
  }

  // A point on the displayed image (CSS pixels from its top-left corner) to
  // the pixels of the camera_info image, and back.
  function displayToImage(x, y, displayWidth, displayHeight, imageWidth, imageHeight) {
    return {x: x * imageWidth / displayWidth, y: y * imageHeight / displayHeight};
  }

  function imageToDisplay(x, y, displayWidth, displayHeight, imageWidth, imageHeight) {
    return {x: x * displayWidth / imageWidth, y: y * displayHeight / imageHeight};
  }

  // The box under an image point: the smallest one containing it (the most
  // specific of overlapping boxes), else the nearest one within
  // tolerance pixels of its edge (small boxes under a finger), else null.
  function hitTest(boxes, x, y, tolerance = 0) {
    let inside = null;
    let near = null;
    for (const box of boxes) {
      const dx = Math.max(Math.abs(x - box.cx) - box.width / 2, 0);
      const dy = Math.max(Math.abs(y - box.cy) - box.height / 2, 0);
      const distance = Math.hypot(dx, dy);
      if (distance === 0) {
        if (!inside || box.width * box.height < inside.width * inside.height) inside = box;
      } else if (distance <= tolerance && (!near || distance < near.distance)) {
        near = {box, distance};
      }
    }
    return inside || (near && near.box) || null;
  }

  // The map's local ENU conversion, about the drone's current fix and
  // navigation odometry (as the map goal has always been sent).
  function latLonToEnu(lat, lon, fix, position) {
    const east = (lon - fix.longitude) * METRES_PER_DEGREE *
      Math.cos(fix.latitude * Math.PI / 180.0);
    const north = (lat - fix.latitude) * METRES_PER_DEGREE;
    return {east: position.x + east, north: position.y + north};
  }

  function enuToLatLon(east, north, fix, position) {
    const lat = fix.latitude + (north - position.y) / METRES_PER_DEGREE;
    const lon = fix.longitude + (east - position.x) /
      (METRES_PER_DEGREE * Math.cos(fix.latitude * Math.PI / 180.0));
    return {lat, lon};
  }

  // The navigation badge, {text, kind} (kind: the badge's ok/warn/bad
  // class). gps: {status, at} from MAVROS' raw GPS fix (NavSatFix status:
  // -1 no fix, 0 or more a fix); correction: {source, at} from the
  // navigation fusion's /navigation/correction_source, which names the
  // vision corrections it applies (demnav, wildnav, demnav+wildnav) and
  // says dead_reckoning whenever none is active, GPS or not: on its own it
  // read "dead_reckoning" while the GPS was healthy. Either is unknown
  // after staleAfterS without a message (times in seconds).
  const CORRECTION_SOURCES = new Set(['demnav', 'wildnav', 'demnav+wildnav']);

  function navigationBadge(gps, correction, nowS, staleAfterS = 5) {
    const fresh = item => Boolean(item) && Number.isFinite(item.at) &&
      nowS - item.at <= staleAfterS;
    const gpsKnown = fresh(gps) && Number.isFinite(gps.status);
    const vision = fresh(correction) && CORRECTION_SOURCES.has(correction.source)
      ? correction.source : null;
    if (gpsKnown && gps.status >= 0) {
      return {text: `Navigation: GPS${vision ? ` + ${vision}` : ''}`, kind: 'ok'};
    }
    if (gpsKnown) {
      return vision ? {text: `Navigation: ${vision} (no GPS)`, kind: 'warn'}
        : {text: 'Navigation: dead reckoning (no GPS)', kind: 'bad'};
    }
    if (vision) return {text: `Navigation: ${vision} (GPS unknown)`, kind: 'warn'};
    if (fresh(correction)) return {text: 'Navigation: GPS unknown', kind: 'warn'};
    return {text: 'Navigation: -', kind: ''};
  }

  // /tracking/status JSON, or {error} when it is not a status object.
  function parseStatus(text) {
    let status;
    try {
      status = JSON.parse(text);
    } catch (error) {
      return {error: `Status is not JSON: ${error.message}`};
    }
    if (!status || typeof status !== 'object' || Array.isArray(status)) {
      return {error: 'Status is not a JSON object'};
    }
    return {status};
  }

  return {
    LIMITS, MAX_MATCH_DT_S, DEFAULT_PREFIX,
    topicNames, prefixFromQuery, parseNumber,
    validateCentered, validateSlant, validateMode, slantRangeBounds, slantGeometry,
    trackCommand, stopCommand,
    stampSeconds, stampDifference, stampKey, parseDetections, DetectionHistory, boxAlpha,
    fitRect, displayToImage, imageToDisplay, hitTest,
    latLonToEnu, enuToLatLon, parseStatus, navigationBadge,
  };
}));
