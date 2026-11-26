/* 深色／淺色背景切換。放在 <head> 內同步載入，避免換頁時閃一下。
   預設深色；使用者的選擇存在 localStorage，全站共用。 */
(function () {
  var KEY = 'theme', root = document.documentElement, cur = null;
  try { cur = localStorage.getItem(KEY); } catch (e) {}
  var q = /[?&]theme=(light|dark)\b/.exec(location.search);
  if (q) cur = q[1];
  if (cur !== 'light' && cur !== 'dark') cur = 'dark';

  function apply(v) {
    cur = v;
    root.setAttribute('data-theme', v);
    var m = document.querySelector('meta[name="theme-color"]');
    if (m) m.setAttribute('content', v === 'light' ? '#f5f6f2' : '#09090b');
    var c = document.querySelector('meta[name="color-scheme"]');
    if (c) c.setAttribute('content', v);
    var b = document.getElementById('themeToggle');
    if (b) {
      var next = v === 'light' ? '深色' : '淺色';
      b.textContent = next;
      b.setAttribute('aria-label', '切換為' + next + '背景');
      b.setAttribute('title', '切換為' + next + '背景');
    }
  }
  apply(cur);

  function mount() {
    apply(cur);
    if (root.classList.contains('embed') || document.getElementById('themeToggle')) return;
    var host = document.querySelector('.nav .wrap') || document.querySelector('.topbar');
    if (!host) return;
    var b = document.createElement('button');
    b.type = 'button'; b.id = 'themeToggle'; b.className = 'theme-tg';
    b.addEventListener('click', function () {
      var v = cur === 'light' ? 'dark' : 'light';
      try { localStorage.setItem(KEY, v); } catch (e) {}
      apply(v);
      try { window.dispatchEvent(new CustomEvent('themechange', { detail: v })); } catch (e) {}
    });
    var btn = host.querySelector('.btn');
    if (btn && btn.parentNode === host) host.insertBefore(b, btn); else host.appendChild(b);
    apply(cur);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', mount); else mount();
  window.addEventListener('storage', function (e) { if (e.key === KEY && (e.newValue === 'light' || e.newValue === 'dark')) apply(e.newValue); });
})();
