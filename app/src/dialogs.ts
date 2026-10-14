// Small in-app dialogs — a name field and a yes/no — so the app never
// depends on the browser's own prompt()/confirm(), which a desktop webview
// may not show.
const esc = (s: string) =>
  s.replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" })[c]!);

/** A small in-app dialog: a text field (`askText`) or yes/no (`askConfirm`). */
function dialog(title: string, body: string, ok: string, danger = false): Promise<HTMLElement | null> {
  return new Promise((resolve) => {
    const back = document.createElement("div");
    back.className = "modal-back";
    back.innerHTML = `<div class="modal"><h3>${esc(title)}</h3>${body}
      <div class="row end"><button data-d="cancel">Cancel</button>
      <button class="${danger ? "danger" : "primary"}" data-d="ok">${esc(ok)}</button></div></div>`;
    const done = (v: HTMLElement | null) => {
      back.remove();
      resolve(v);
    };
    back.addEventListener("click", (e) => {
      const d = (e.target as HTMLElement).dataset.d;
      if (e.target === back || d === "cancel") done(null);
      if (d === "ok") done(back);
    });
    back.addEventListener("keydown", (e) => {
      if (e.key === "Enter") done(back);
      if (e.key === "Escape") done(null);
    });
    document.body.appendChild(back);
    (back.querySelector("input") ?? back.querySelector<HTMLElement>("[data-d=ok]"))?.focus();
  });
}

export async function askText(title: string, label: string, value: string, ok = "OK"): Promise<string | null> {
  const el = await dialog(title, `<label>${esc(label)}<input id="ask-text" value="${esc(value)}" /></label>`, ok);
  const v = el?.querySelector<HTMLInputElement>("#ask-text")?.value.trim();
  return v ? v : null;
}

export async function askConfirm(title: string, text: string, ok: string): Promise<boolean> {
  return !!(await dialog(title, `<p class="hint" style="font-size:13px">${esc(text)}</p>`, ok, true));
}

