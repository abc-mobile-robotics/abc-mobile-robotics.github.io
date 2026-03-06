---
layout: default
title: Wolf and Rabbit
---

<div class="hero-side-images">
  <div class="image-box">
    <img src="{{ '/mobile-robotics/assets//images/wolf.png' | relative_url }}" alt="Wolf" class="side-image">
  </div>

  <div class="hero-center">
    <h1>Wolf and Rabbit</h1>
    <p class="hero-subtitle">
      A ROS 2 multi-robot pursuit project using two TurtleBots, vision sensing, and distance measurement.
    </p>

    <div class="hero-buttons">
      <a class="custom-button" href="https://aldrickpeter.github.io/mobile-robotics/">Visit Website</a>
      <a class="custom-button secondary-button" href="https://github.com/AldrickPeter/mobile-robotics">View Repository</a>
    </div>

    <div class="commit-box">
      <span class="commit-label">7-character short hash:</span>
      <code>{{ site.github.build_revision | slice: 0, 7 }}</code>
    </div>
  </div>

  <div class="image-box">
    <img src="{{ '/mobile-robotics/assets/images/rabbit.png' | relative_url }}" alt="Rabbit" class="side-image">
  </div>
</div>

## Project Description

Our project, **Wolf and Rabbit**, uses two TurtleBot robots to simulate a pursuit scenario.  
One robot acts as the **Wolf**, which detects and chases the target, while the other acts as the **Rabbit**, which moves as the target to escape.

This system integrates perception, communication, and autonomous motion control in a ROS 2 environment.

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

## Main Features

- **Wolf**: detects and chases the rabbit
- **Rabbit**: acts as the moving target
- **ROS 2** for communication and robot control
- **Vision sensors** for target detection and tracking
- **Distance sensors** for measuring relative distance

## Required Items

### 1. Link to our GitHub Pages
[Visit our GitHub Pages site](https://aldrickpeter.github.io/mobile-robotics/)

### 2. 7-character short hash of our commit
`{{ site.github.build_revision | slice: 0, 7 }}`

## GitHub Repository
[mobile-robotics repository](https://github.com/AldrickPeter/mobile-robotics)