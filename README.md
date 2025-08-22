# Multi-Robot Shortest Path Warehouse Simulator

**Implementation Date:** November 2024  
**Language & Framework:** C++ with STL and Object-Oriented Programming

## Overview

This project simulates a warehouse environment featuring racks, obstacles, and multiple autonomous robots. It highlights real-time multi-robot path planning with collision avoidance and integrates a scheduler that benchmarks different pathfinding algorithms for performance comparison.

## Features

- **Warehouse Simulation Environment**  
  - Simulates shelving racks and obstacles in a grid-like warehouse.
  - Supports multiple robots navigating concurrently.

- **Pathfinding Algorithms Implemented**  
  - A\* (A-Star) algorithm for efficient heuristic-driven pathfinding.  
  - Dijkstra’s algorithm for classic shortest-path comparison.

- **Scheduler & Collision Avoidance**  
  - Manages robot movements to minimize path conflicts and collisions.
  - Prioritizes robot scheduling based on path outcomes.

- **Performance Benchmarking**  
  - Benchmarks A\* vs Dijkstra with scenarios involving 5+ robots.
  - Provides real-time comparisons of path length, runtime efficiency, and overall throughput.

## Getting Started

### Prerequisites

- C++17 compatible compiler (e.g., `g++`, `clang++`)
- Standard C++ Library support

### Building the Project

```bash
g++ -std=c++17 Main.cpp Asssignment2.cpp -o multi_robot_simulator
