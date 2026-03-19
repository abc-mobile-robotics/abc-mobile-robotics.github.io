---
layout: default
title: Wolf and Rabbit
---

<div class="page-layout">


  <main class="main-content-area">
    <div class="hero-side-images">
      <div class="image-box">
        <img src="{{ '/assets/images/wolf.png' | relative_url }}" alt="Wolf" class="side-image">
      </div>

      <div class="hero-center">
        <h1>Wolf and Rabbit</h1>
        <p class="hero-subtitle">
          A ROS 2 multi-robot pursuit project using two TurtleBots, vision sensing, and distance measurement.
        </p>

        <div class="hero-buttons">
          <a class="custom-button" href="https://abc-mobile-robotics.github.io/">Visit Website</a>
          <a class="custom-button secondary-button" href="https://github.com/AldrickPeter/mobile-robotics">View Repository</a>
        </div>

        <div class="commit-box">
          <span class="commit-label">7-character short hash:</span>
          <code>{{ site.github.build_revision | slice: 0, 7 }}</code>
        </div>
      </div>

      <div class="image-box">
        <img src="{{ '/assets/images/rabbit.png' | relative_url }}" alt="Rabbit" class="side-image">
      </div>
    </div>

    <h2 id="overview">Project Description</h2>

    <p>
      Our project, <b>Wolf and Rabbit</b>, uses two TurtleBot robots to simulate a pursuit scenario.
      One robot acts as the <b>Wolf</b>, which detects and chases the target, while the other acts as the <b>Rabbit</b>, which moves as the target to escape.
    </p>

    <p>
      This system integrates perception, communication, and autonomous motion control in a ROS 2 environment.
    </p>

    <h2 id="team">Get to know us</h2>

    <div style="display:flex; justify-content:center; gap:32px; flex-wrap:wrap; margin:20px 0 36px 0;">
      <div style="text-align:center; width:220px;">
        <div style="width:220px; height:220px; margin:0 auto; overflow:hidden; border-radius:16px; box-shadow:0 6px 18px rgba(0,0,0,0.12); background:#f3f4f6;">
          <img src="{{ '/assets/images/brian.jpg' | relative_url }}" alt="Brian"
              style="width:100%; height:100%; object-fit:cover; display:block;">
        </div>
        <p style="margin-top:12px; margin-bottom:0; font-weight:700; font-size:1.05rem; text-align:center;">Brian</p>
      </div>

      <div style="text-align:center; width:220px;">
        <div style="width:220px; height:220px; margin:0 auto; overflow:hidden; border-radius:16px; box-shadow:0 6px 18px rgba(0,0,0,0.12); background:#f3f4f6;">
          <img src="{{ '/assets/images/chach.jpg' | relative_url }}" alt="Chach"
              style="width:100%; height:100%; object-fit:cover; display:block;">
        </div>
        <p style="margin-top:12px; margin-bottom:0; font-weight:700; font-size:1.05rem; text-align:center;">Chach</p>
      </div>

      <div style="text-align:center; width:220px;">
        <div style="width:220px; height:220px; margin:0 auto; overflow:hidden; border-radius:16px; box-shadow:0 6px 18px rgba(0,0,0,0.12); background:#f3f4f6;">
          <img src="{{ '/assets/images/aldrick.jpg' | relative_url }}" alt="Aldrick"
              style="width:100%; height:100%; object-fit:cover; display:block;">
        </div>
        <p style="margin-top:12px; margin-bottom:0; font-weight:700; font-size:1.05rem; text-align:center;">Aldrick</p>
      </div>
    </div>

    <div class="feature-grid">
      <div class="feature-card">
        <h3>ROS 2 Control</h3>
        <p>ROS 2 is used for communication, coordination, and robot control between the two TurtleBots.</p>
      </div>

      <div class="feature-card">
        <h3>Vision Sensing</h3>
        <p>Vision sensors are used for target detection and tracking during the pursuit process.</p>
      </div>

      <div class="feature-card">
        <h3>Distance Measurement</h3>
        <p>Distance sensors help estimate the relative distance and support real-time chase behavior.</p>
      </div>
    </div>

    <h2 id="features">Main Features</h2>

    <ul>
      <li><b>Wolf</b>: detects and chases the rabbit</li>
      <li><b>Rabbit</b>: acts as the moving target</li>
      <li><b>ROS 2</b> for communication and robot control</li>
      <li><b>Vision sensors</b> for target detection and tracking</li>
      <li><b>Distance sensors</b> for measuring relative distance</li>
    </ul>

    <h2 id="required">Required Items</h2>

    <h3>1. Link to our GitHub Pages</h3>
    <p>
      <a href="https://abc-mobile-robotics.github.io/">Visit our GitHub Pages site</a>
    </p>

    <h3>2. 7-character short hash of our commit</h3>
    <p>
      <code>{{ site.github.build_revision | slice: 0, 7 }}</code>
    </p>

    <h2 id="repo">GitHub Repository</h2>
    <p>
      <a href="https://github.com/AldrickPeter/mobile-robotics">mobile-robotics repository</a>
    </p>
  </main>
</div>