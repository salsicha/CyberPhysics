// On-screen joysticks of the RACECAR web UI (main.py), for touch and mouse.
//
// Any element with data-joystick="<name>" is a pad; its .joy-knob child
// follows the pointer. While a finger or the mouse holds a pad, the stick
// (x right, y up, inside the unit disc) goes to the server every 100 ms as
// emitEvent('joystick', {name, x, y, held: true, t}), and once more with
// held: false on release, when the pointer is lost, or when the page goes to
// the background. t (seconds, this page's monotonic clock) lets the server
// drop reports that were buffered while the connection was down. The server
// repeats commands only while reports keep arriving
// (racecar_ui_logic.Hold), so a frozen or disconnected tab stops sending.
(() => {
  const PERIOD_MS = 100;
  const holds = new Map(); // pointerId -> {pad, name, x, y, timer}

  function report(hold, held) {
    emitEvent("joystick", { name: hold.name, x: hold.x, y: hold.y, held, t: performance.now() / 1000 });
  }

  function knob(pad) {
    return pad.querySelector(".joy-knob");
  }

  function place(hold, event) {
    const rect = hold.pad.getBoundingClientRect();
    const reach = (rect.width / 2) * 0.72; // knob travel in pixels
    let x = (event.clientX - rect.left - rect.width / 2) / reach;
    let y = -(event.clientY - rect.top - rect.height / 2) / reach;
    const norm = Math.hypot(x, y);
    if (norm > 1) {
      x /= norm;
      y /= norm;
    }
    hold.x = x;
    hold.y = y;
    const element = knob(hold.pad);
    if (element) element.style.transform = `translate(${x * reach}px, ${-y * reach}px)`;
  }

  function release(pointerId) {
    const hold = holds.get(pointerId);
    if (!hold) return;
    holds.delete(pointerId);
    clearInterval(hold.timer);
    hold.x = 0;
    hold.y = 0;
    const element = knob(hold.pad);
    if (element) element.style.transform = "";
    hold.pad.classList.remove("joy-active");
    report(hold, false);
  }

  document.addEventListener("pointerdown", (event) => {
    const pad = event.target.closest ? event.target.closest("[data-joystick]") : null;
    if (!pad) return;
    event.preventDefault();
    for (const [id, other] of holds) if (other.pad === pad) release(id); // one pointer per pad
    try {
      pad.setPointerCapture(event.pointerId);
    } catch (e) {
      // Synthetic events have no active pointer to capture.
    }
    const hold = { pad, name: pad.dataset.joystick, x: 0, y: 0 };
    holds.set(event.pointerId, hold);
    pad.classList.add("joy-active");
    place(hold, event);
    report(hold, true);
    hold.timer = setInterval(() => report(hold, true), PERIOD_MS);
  });
  document.addEventListener(
    "pointermove",
    (event) => {
      const hold = holds.get(event.pointerId);
      if (!hold) return;
      event.preventDefault();
      place(hold, event);
    },
    { passive: false },
  );
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    document.addEventListener(type, (event) => release(event.pointerId));
  }
  const releaseAll = () => {
    for (const id of [...holds.keys()]) release(id);
  };
  window.addEventListener("blur", releaseAll);
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) releaseAll();
  });
})();
