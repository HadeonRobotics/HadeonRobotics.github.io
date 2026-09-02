const NAV_HTML = `
<header class="site-header">
  <div class="container nav-bar">
    <a href="/" class="brand">
      <img src="/images/HadeonLogo.svg" alt="Hadeon Robotics">
    </a>
    <input type="checkbox" id="nav-toggle" class="nav-toggle-input">
    <label for="nav-toggle" class="nav-hamburger" aria-label="Toggle navigation">
      <span></span><span></span><span></span>
    </label>
    <nav class="nav-links">
      <a href="/solutions.html" data-page="solutions">Solutions</a>
      <a href="/about.html" data-page="about">About</a>
      <a href="/news.html" data-page="news">News</a>
      <a href="/careers.html" data-page="careers">Careers</a>
    </nav>
    <a href="mailto:info@hadeonrobotics.com?subject=Demo%20Request" class="btn btn-primary blueprint nav-book-demo">
      Book a demo
      <i class="corner tl"></i><i class="corner tr"></i><i class="corner bl"></i><i class="corner br"></i>
    </a>
  </div>
</header>`;

const FOOTER_HTML = `
<footer class="site-footer" id="contact">
  <div class="container footer-grid">
    <div class="footer-brand">
      <img src="/images/HadeonLogo.svg" alt="Hadeon Robotics">
      <p>The deterministic motion engine for industrial robots.<br>New Brunswick, Canada.</p>
    </div>
    <div>
      <div class="footer-heading">Site</div>
      <div class="footer-links">
        <a href="/">Home</a>
        <a href="/solutions.html">Solutions</a>
        <a href="/about.html">About</a>
        <a href="/news.html">News</a>
        <a href="/careers.html">Careers</a>
      </div>
    </div>
    <div>
      <div class="footer-heading">Contact</div>
      <div class="footer-links">
        <a href="mailto:info@hadeonrobotics.com">info@hadeonrobotics.com</a>
        <a href="https://www.linkedin.com/company/hadeon-robotics/" target="_blank" rel="noopener">LinkedIn</a>
        <a href="https://github.com/Dynamium-Lab/blast" target="_blank" rel="noopener">BLAST on GitHub</a>
      </div>
    </div>
  </div>
  <div class="container footer-bottom">&copy; 2026 Hadeon Robotics Inc. All rights reserved.</div>
</footer>`;

function injectComponents() {
    const navMount = document.getElementById('nav-mount');
    if (navMount) navMount.outerHTML = NAV_HTML;

    const footerMount = document.getElementById('footer-mount');
    if (footerMount) footerMount.outerHTML = FOOTER_HTML;

    const page = document.body.getAttribute('data-page');
    if (page) {
        const activeLink = document.querySelector(`.nav-links a[data-page="${page}"]`);
        if (activeLink) activeLink.classList.add('active');
    }
}

injectComponents();
