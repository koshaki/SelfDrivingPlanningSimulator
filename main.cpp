#include "world.h"

#include <iostream>
#include <fstream>
#include <tuple>
#include <cassert>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>

using namespace std;

World world;
double a, df;
mutex mtx;
bool pressed;


template <class Value>
int Sign(Value Val) {
  if (Val > 0.)  return 1;
  else return -1;
}


double getAcceleration(const Car& car) {
  if ((car.psi >= 0.2 or car.psi <= -0.2) and car.v >= 10) {
    return -100;
  }
  if ((car.frontAngle >= 0.2 or car.frontAngle <= -0.2) and car.v >= 10) {
    return -100;
  }
  auto crosswalks = world.getCrosswalks();

  std::sort(std::begin(crosswalks), std::end(crosswalks), [](const Crosswalk& lhs, const Crosswalk& rhs) {
    return lhs.lx < rhs.lx;
  });

  const Crosswalk* nearestCrosswalk = nullptr;
  for (const auto& cr : crosswalks) {
    if (cr.lx > car.x + Car::LENGTH) {
      nearestCrosswalk = &cr;
      break;
    }
  }


  const double maxDeceleration = 100;
  const double brakingDistance = car.v * car.v / maxDeceleration;

  const auto peds = world.getPedestrians();
  for (const auto& ped : peds) {
    const double dir_y = sin(ped.yaw) < 0 ? -1 : 1;

    const bool is_pedestrian_on_crosswalk = (
      ped.x + PedestrianFearRadius > nearestCrosswalk->lx &&
      ped.x - PedestrianFearRadius < nearestCrosswalk->rx);

    const bool is_on_same_lane = (
      ped.y + PedestrianFearRadius > car.y - Car::WIDTH &&
      ped.y - PedestrianFearRadius < car.y + Car::WIDTH);

    const bool is_moving_to_our_lane = (
      (ped.y > car.y && dir_y < 0) || (ped.y < car.y && dir_y > 0));

    if ((is_pedestrian_on_crosswalk) &&
        (is_on_same_lane || is_moving_to_our_lane) and (car.x + Car::LENGTH + PedestrianFearRadius >= nearestCrosswalk->lx - brakingDistance)) {
       return -100;
    }
  }
  auto obstacles = world.getObstacles();

  std::sort(std::begin(obstacles), std::end(obstacles), [](const Obstacle& lhs, const Obstacle& rhs) {
    return lhs.x < rhs.x;
  });

  const Obstacle* obs = nullptr;
  for (const auto& cr : obstacles) {
    if (cr.x > car.x + Car::LENGTH) {
      obs = &cr;
      break;
    }
  }
  double top_car = car.y - Car::WIDTH / 2 - 3;
  double bottom_car = 3 + car.y + Car::WIDTH / 2;

  double top_obs = obs->y - obs->r;
  double bottom_obs = obs->y + obs->r;


  const bool obstacle_in_front = ((top_car <= bottom_obs && top_car >= top_obs) || (bottom_car >= top_obs && bottom_car <= bottom_obs));
  const bool obstacle_near = (obs->x > car.x && obs->x - car.x < 200);
  double distance = obs->x - car.x;

  if (car.v >= 25 && obstacle_near && obstacle_in_front) {
    return -10;
  }
  if (car.v >= 60) {
    return -8000000 / distance;
  }
  
  return 100;
}

double getDF(const Car& car) {
  
  auto obstacles = world.getObstacles();

  std::sort(std::begin(obstacles), std::end(obstacles), [](const Obstacle& lhs, const Obstacle& rhs) {
    return lhs.x < rhs.x;
  });

  const Obstacle* obs = nullptr;
  for (const auto& cr : obstacles) {
    if (cr.x > car.x + Car::LENGTH) {
      obs = &cr;
      break;
    }
  }
  double top_car = car.y - Car::WIDTH / 2;
  double bottom_car = car.y + Car::WIDTH / 2;

  double top_obs = obs->y - obs->r;
  double bottom_obs = obs->y + obs->r;

  bool position = car.y < 55;
  double psi = car.psi;
  const bool obstacle_in_front = ((top_car <= bottom_obs && top_car >= top_obs) || (bottom_car >= top_obs && bottom_car <= bottom_obs));
  const bool obstacle_near = (obs->x > car.x && obs->x - car.x < 200);

  if (obstacle_in_front && obstacle_near) {
    if (position) {
      return 100;
    }
    else {
      return -100;
    }
  }
  else {
    return -0.5 * car.psi;
  }
}

void solve() {
    while (true) {
        mtx.lock();
        world.updatePedestrians();

        [[maybe_unused]] const Car ego = world.getMyCar();
        double a = getAcceleration(ego);
        double df = getDF(ego);

        world.makeStep(a, df);

        if (!pressed) {
            a = 0, df = 0;
        }
        pressed = false;

        if (world.checkCollisions()) {
            cerr << "collision in " << world.getTimeSinceStart() << " sec.\n";
            world.init();
        }
        if (world.gameOver()) {
            cerr << world.getTimeSinceStart() << " sec.\n";
            break;
        }
        mtx.unlock();

        this_thread::sleep_for(chrono::milliseconds(10));
    }
}


int main(int /*argc*/, char ** /*argv*/) {
    srand(1991);
    cout << "Started solution" << endl;

    v.setSize(W, H);

    v.setOnKeyPress([&](const QKeyEvent& ev) {
        pressed = true;
        if (ev.key() == Qt::Key_W) a = 100;
        if (ev.key() == Qt::Key_S) a = -100;
        if (ev.key() == Qt::Key_A) df = -0.8;
        if (ev.key() == Qt::Key_D) df = 0.8;
    });

    world.init();
    thread solveThread(solve);

    World world_copy;
    while (true) {
        RenderCycle r(v);

        {
            mtx.lock();
            world_copy = world;
            mtx.unlock();
        }
        world_copy.draw();
    }
}
