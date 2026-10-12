// Unit tests of the aerodrone web UI's logic
// (applications/aerostack2/src/as2_platform_blueos/web/tracking_logic.js),
// run with node by tests/test_aerodrone_tracking_ui.py: `node <this file>`.
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const test = require('node:test');

const repo = path.resolve(__dirname, '..', '..');
const T = require(path.join(
  repo, 'applications/aerostack2/src/as2_platform_blueos/web/tracking_logic.js'));
const contract = fs.readFileSync(path.join(repo, 'systems/aerodrone/tracking.md'), 'utf8');

function contractRow(topic) {
  const row = contract.split('\n').find(line => line.startsWith(`| \`${topic}\``));
  assert.ok(row, `tracking.md has a row for ${topic}`);
  return row;
}

test('topic names are the contract topics under the prefix', () => {
  const names = T.topicNames(T.prefixFromQuery(''));
  for (const name of Object.values(names)) contractRow(name);
  assert.deepEqual(T.topicNames('/drone2/tracking').command, '/drone2/tracking/command');
  assert.equal(T.prefixFromQuery('?prefix=/drone2/tracking'), '/drone2/tracking');
  for (const bad of ['?prefix=tracking', '?prefix=/a b', '?prefix=/x/', '?prefix=']) {
    assert.equal(T.prefixFromQuery(bad), '/tracking', bad);
  }
});

test('commands equal the contract examples', () => {
  const examples = [...contractRow('/tracking/command').matchAll(/`(\{[^`]*\})`/g)]
    .map(match => JSON.parse(match[1]));
  assert.equal(examples.length, 3);
  const [centered, slant, stop] = examples;
  assert.deepEqual(T.trackCommand('7', 'centered', {altitude: '40'}).command, centered);
  assert.deepEqual(
    T.trackCommand('7', 'slant', {slantRange: '60', depression: '70'}).command, slant);
  assert.deepEqual(T.stopCommand().command, stop);
  assert.equal(T.stopCommand().json, '{"action":"stop"}');
  const json = T.trackCommand(7, 'centered', {altitude: ' 40 '}).json;
  assert.equal(json, '{"action":"track","target_id":"7","mode":"centered","altitude_agl_m":40}');
});

test('centered mode: empty height is left out, limits are 15-120 m', () => {
  assert.deepEqual(T.trackCommand('3', 'centered', {altitude: ''}).command,
                   {action: 'track', target_id: '3', mode: 'centered'});
  for (const ok of ['15', '120', '62.5']) {
    assert.equal(T.validateCentered(ok).ok, true, ok);
  }
  for (const bad of ['14.9', '120.1', '-40', 'abc', 'Infinity', '1e9']) {
    const result = T.validateCentered(bad);
    assert.equal(result.ok, false, bad);
    assert.match(result.error, /Height above ground/);
  }
});

test('slant mode: depression 60-90 deg, height above the target 15-120 m', () => {
  for (const depression of ['60', '75', '90']) {
    assert.equal(T.validateSlant('60', depression).ok, true, depression);
  }
  for (const depression of ['59.9', '90.1', '', 'x']) {
    const result = T.validateSlant('60', depression);
    assert.equal(result.ok, false, depression);
    assert.match(result.error, /Depression angle/);
  }
  const bounds = T.slantRangeBounds(70);
  assert.ok(Math.abs(bounds.min - 15 / Math.sin(70 * Math.PI / 180)) < 1e-9);
  assert.ok(Math.abs(bounds.max - 120 / Math.sin(70 * Math.PI / 180)) < 1e-9);
  assert.equal(T.validateSlant(bounds.min.toFixed(1), '70').ok, true);
  assert.equal(T.validateSlant(bounds.max.toFixed(1), '70').ok, true);
  assert.equal(T.validateSlant('15', '70').ok, false);
  assert.match(T.validateSlant('200', '70').error, /Slant range must be 16\.0-127\.7 m/);
  for (const range of ['', '-5', 'NaN']) {
    assert.equal(T.validateSlant(range, '70').ok, false, range);
  }
  const geometry = T.slantGeometry(60, 70);
  assert.ok(Math.abs(geometry.standoffM - 20.521) < 1e-3);
  assert.ok(Math.abs(geometry.heightM - 56.382) < 1e-3);
  assert.equal(T.validateMode('orbit', {}).ok, false);
});

test('an object without a track ID cannot be followed', () => {
  for (const id of ['', null, undefined]) {
    assert.equal(T.trackCommand(id, 'centered', {altitude: ''}).ok, false);
  }
});

function detections(sec, nanosec, boxes) {
  return {
    header: {stamp: {sec, nanosec}, frame_id: 'camera'},
    detections: boxes.map(([id, cls, cx, cy, w, h, score]) => ({
      id, bbox: {center: {position: {x: cx, y: cy}, theta: 0}, size_x: w, size_y: h},
      results: [{hypothesis: {class_id: cls, score}, pose: {}}],
    })),
  };
}

test('detections parse from rosbridge vision_msgs JSON', () => {
  const boxes = T.parseDetections(detections(5, 0, [['7', 'car', 100, 50, 40, 20, 0.8]]));
  assert.deepEqual(boxes, [{id: '7', className: 'car', score: 0.8, cx: 100, cy: 50,
                            width: 40, height: 20}]);
  const message = detections(5, 0, [['1', 'car', 1, 1, 1, 1, 0.5]]);
  delete message.detections[0].results;
  message.detections.push({id: '2', bbox: {center: {position: {x: 'a'}}}});
  assert.deepEqual(T.parseDetections(message),
                   [{id: '1', className: '', score: null, cx: 1, cy: 1, width: 1, height: 1}]);
  assert.deepEqual(T.parseDetections({}), []);
});

test('a frame gets its own detections, else the nearest within 0.5 s', () => {
  const history = new T.DetectionHistory();
  history.add(detections(100, 0, [['1', 'car', 10, 10, 4, 4, 0.9]]));
  history.add(detections(100, 200000000, [['1', 'car', 12, 10, 4, 4, 0.9]]));
  history.add(detections(100, 400000000, [['1', 'car', 14, 10, 4, 4, 0.9]]));
  const exact = history.match({sec: 100, nanosec: 200000000});
  assert.equal(exact.exact, true);
  assert.equal(exact.boxes[0].cx, 12);
  const near = history.match({sec: 100, nanosec: 250000000});
  assert.equal(near.exact, false);
  assert.equal(near.boxes[0].cx, 12);
  assert.ok(Math.abs(near.dt - 0.05) < 1e-6);
  const tie = history.match({sec: 100, nanosec: 300000000});
  assert.equal(tie.boxes[0].cx, 14, 'the later detections on a tie');
  assert.equal(history.match({sec: 100, nanosec: 900000001}), null);
  assert.equal(history.match({sec: 99, nanosec: 600000000}).boxes[0].cx, 10);
  assert.equal(history.match({sec: 99, nanosec: 400000000}), null);
  assert.equal(new T.DetectionHistory().match({sec: 1, nanosec: 0}), null);
});

test('the history forgets old stamps and restarts after a clock jump back', () => {
  const history = new T.DetectionHistory(5, 100);
  for (let i = 0; i < 80; i += 1) {
    history.add(detections(1000 + Math.floor(i / 10), (i % 10) * 1e8, []));
  }
  assert.ok(history.entries.length <= 51);
  history.add(detections(10, 0, [['9', 'person', 1, 1, 1, 1, 0.5]]));
  assert.equal(history.entries.length, 1);
  assert.equal(history.match({sec: 10, nanosec: 0}).boxes[0].id, '9');
});

test('stale boxes fade', () => {
  assert.equal(T.boxAlpha(0, 0), 1);
  assert.ok(T.boxAlpha(0.25, 0) < 1 && T.boxAlpha(0.25, 0) > T.boxAlpha(0.5, 0));
  assert.equal(T.boxAlpha(0.5, 0), 0.35);
  assert.equal(T.boxAlpha(0, 3), 0.5);
});

test('display <-> image pixels and fitting keep the aspect ratio', () => {
  const fit = T.fitRect(1000, 500, 640, 480);
  assert.ok(Math.abs(fit.width - 500 * 640 / 480) < 1e-9 && Math.abs(fit.height - 500) < 1e-9);
  assert.deepEqual(T.fitRect(320, 900, 640, 480), {width: 320, height: 240});
  assert.deepEqual(T.fitRect(0, 100, 640, 480), {width: 0, height: 0});
  const image = T.displayToImage(160, 60, 320, 240, 640, 480);
  assert.deepEqual(image, {x: 320, y: 120});
  assert.deepEqual(T.imageToDisplay(image.x, image.y, 320, 240, 640, 480), {x: 160, y: 60});
});

test('hit testing picks the smallest box under the point, then the nearest', () => {
  const big = {id: 'big', cx: 100, cy: 100, width: 100, height: 100};
  const small = {id: 'small', cx: 110, cy: 110, width: 10, height: 10};
  const far = {id: 'far', cx: 300, cy: 100, width: 6, height: 6};
  const boxes = [big, small, far];
  assert.equal(T.hitTest(boxes, 110, 110).id, 'small');
  assert.equal(T.hitTest(boxes, 60, 60).id, 'big');
  assert.equal(T.hitTest(boxes, 310, 100), null);
  assert.equal(T.hitTest(boxes, 310, 100, 8).id, 'far');
  assert.equal(T.hitTest(boxes, 400, 400, 8), null);
  assert.equal(T.hitTest([], 1, 1, 100), null);
});

test('ENU <-> map latitude/longitude round trip, about the fix and odometry', () => {
  const fix = {latitude: 37.9234, longitude: -122.5967};
  const position = {x: 120.0, y: -40.0};
  const enu = T.latLonToEnu(37.9244, -122.5957, fix, position);
  assert.ok(Math.abs(enu.north - (-40 + 0.001 * 111320)) < 1e-6);
  const back = T.enuToLatLon(enu.east, enu.north, fix, position);
  assert.ok(Math.abs(back.lat - 37.9244) < 1e-12);
  assert.ok(Math.abs(back.lon - -122.5957) < 1e-12);
  const here = T.enuToLatLon(position.x, position.y, fix, position);
  assert.deepEqual(here, {lat: fix.latitude, lon: fix.longitude});
});

test('status parsing', () => {
  assert.deepEqual(T.parseStatus('{"state": "tracking", "target_id": "7"}').status,
                   {state: 'tracking', target_id: '7'});
  assert.match(T.parseStatus('nope').error, /not JSON/);
  assert.match(T.parseStatus('[1]').error, /not a JSON object/);
  assert.match(T.parseStatus('null').error, /not a JSON object/);
});
