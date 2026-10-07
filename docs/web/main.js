/* Entry point: one virtual bus shared by the dashboard, the bench and the 3D car.
 * Browser counterpart of ``python dashboard/stm32_dashboard.py --sim --3d``.
 */
(function () {
  "use strict";

  const S = window.CanSim;

  // Auto sweep and random button presses on, so the page is alive before the
  // visitor touches anything; moving a slider takes that sensor back to manual.
  const bus = new S.VirtualBus(true, true);

  const dashboard = new S.Dashboard(document.getElementById("view-dashboard"), bus);
  new S.VirtualBench(document.getElementById("view-bench"), bus);
  new S.Car3D(document.getElementById("view-car"), bus);

  bus.start();
  dashboard.connect();

  // ── tabs ──────────────────────────────────────────────────
  const tabs = Array.from(document.querySelectorAll(".tabs [data-view]"));

  function show(name) {
    if (!tabs.some((t) => t.dataset.view === name)) name = "dashboard";
    for (const t of tabs) {
      const on = t.dataset.view === name;
      t.classList.toggle("active", on);
      t.setAttribute("aria-selected", on);
      document.getElementById(`view-${t.dataset.view}`).classList.toggle("active", on);
    }
  }

  for (const t of tabs) {
    t.addEventListener("click", () => {
      history.replaceState(null, "", `#${t.dataset.view}`);
      show(t.dataset.view);
    });
  }
  window.addEventListener("hashchange", () => show(location.hash.slice(1)));
  show(location.hash.slice(1));
})();
