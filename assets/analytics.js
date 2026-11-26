/* 全站流量追蹤（全部免費）
   - Google Analytics 4：來源、頁面、停留時間、自訂事件
   - Microsoft Clarity：熱點圖與匿名操作錄影
   本機開檔（file://）或 localhost 不載入，避免測試灌水。
   後台：
   - GA4      https://analytics.google.com/ （帳戶「林泓佑作品集」，資源 pomeloouo.github.io）
   - Clarity  https://clarity.microsoft.com/projects/view/zjrbkifivs/dashboard
*/
(function () {
  var GA4_ID = 'G-D5K85H5YWP';
  var CLARITY_ID = 'zjrbkifivs';

  var host = location.hostname;
  if (!/^https?:$/.test(location.protocol) || host === 'localhost' || host === '127.0.0.1') return;
  /* 被其他頁面用 iframe 嵌入時（例如 9PIN 模型嵌在案件頁）不重複計算 */
  if (window.top !== window.self) return;

  /* ---------- GA4 ---------- */
  var s = document.createElement('script');
  s.async = true;
  s.src = 'https://www.googletagmanager.com/gtag/js?id=' + GA4_ID;
  document.head.appendChild(s);
  window.dataLayer = window.dataLayer || [];
  function gtag() { dataLayer.push(arguments); }
  window.gtag = gtag;
  gtag('js', new Date());
  gtag('config', GA4_ID, { page_title: document.title });

  /* ---------- Clarity ---------- */
  (function (c, l, a, r, i, t, y) {
    c[a] = c[a] || function () { (c[a].q = c[a].q || []).push(arguments); };
    t = l.createElement(r); t.async = 1; t.src = 'https://www.clarity.ms/tag/' + i;
    y = l.getElementsByTagName(r)[0]; y.parentNode.insertBefore(t, y);
  })(window, document, 'clarity', 'script', CLARITY_ID);

  /* ---------- 共用：送事件到 GA4，同時在 Clarity 標記，方便篩選錄影 ---------- */
  var page = location.pathname.replace(/\/$/, '/index') || '/index';
  function track(name, params) {
    params = params || {};
    params.page_path = page;
    gtag('event', name, params);
    try { window.clarity('event', name); } catch (e) {}
  }
  window.trackEvent = track;
  function text(el) { return (el.innerText || el.textContent || el.getAttribute('aria-label') || '').replace(/\s+/g, ' ').trim().slice(0, 60); }

  /* ---------- 點擊：聯絡方式、按鈕、案件卡、站內連結 ---------- */
  var last = {};
  document.addEventListener('click', function (e) {
    var a = e.target.closest && e.target.closest('a, button, summary');
    if (!a) return;
    var href = a.getAttribute('href') || '';
    var label = text(a);
    var key = href + '|' + label, now = Date.now();
    if (last[key] && now - last[key] < 1500) return;   /* 連點只算一次 */
    last[key] = now;

    /* 聯絡點擊：contact_click 記錄方式；generate_lead 是 GA4 裡設定的「重要事件」（每次瀏覽只算一次） */
    var method = /^https:\/\/line\.me\//.test(href) ? 'line' : /^mailto:/.test(href) ? 'email' : /^tel:/.test(href) ? 'phone' : '';
    if (method) {
      track('contact_click', { method: method, link_text: label });
      gtag('event', 'generate_lead', { method: method, page_path: page });
      return;
    }

    if (a.classList.contains('case')) {
      var h = a.querySelector('h3');
      return track('case_click', { case_name: h ? text(h) : label, link_url: href });
    }
    if (a.tagName === 'SUMMARY') return track('faq_open', { question: label });
    if (a.tagName === 'BUTTON') return track('ui_click', { label: label || a.id || 'button' });
    if (a.classList.contains('btn') || a.classList.contains('nav-cta')) return track('cta_click', { label: label, link_url: href });
    if (href.charAt(0) === '#') return track('nav_click', { target: href, label: label });
  }, true);

  /* ---------- 捲動深度 25 / 50 / 75 / 100% ---------- */
  var marks = [25, 50, 75, 100], sent = {};
  var ticking = false;
  function onScroll() {
    if (ticking) return; ticking = true;
    requestAnimationFrame(function () {
      ticking = false;
      var h = document.documentElement;
      var max = h.scrollHeight - h.clientHeight;
      var pct = max > 0 ? Math.round((h.scrollTop || document.body.scrollTop) / max * 100) : 100;
      for (var i = 0; i < marks.length; i++) {
        if (pct >= marks[i] && !sent[marks[i]]) { sent[marks[i]] = 1; track('scroll_depth', { percent: marks[i] }); }
      }
    });
  }
  addEventListener('scroll', onScroll, { passive: true });

  /* ---------- 區塊曝光：每個有 id 的 section 看到超過一半才算一次 ---------- */
  if ('IntersectionObserver' in window) {
    var seen = {};
    var io = new IntersectionObserver(function (es) {
      es.forEach(function (en) {
        var id = en.target.id;
        if (en.isIntersecting && !seen[id]) { seen[id] = 1; track('section_view', { section: id }); io.unobserve(en.target); }
      });
    }, { threshold: 0.5 });
    document.addEventListener('DOMContentLoaded', function () {
      document.querySelectorAll('section[id]').forEach(function (el) { io.observe(el); });
    });
  }

  /* ---------- 互動式 3D 與模擬器：第一次操作畫布時記一次 ---------- */
  var touched = false;
  document.addEventListener('pointerdown', function (e) {
    if (touched || !e.target || e.target.tagName !== 'CANVAS') return;
    touched = true;
    track('interactive_use', { element: e.target.id || 'canvas' });
  }, true);
})();
