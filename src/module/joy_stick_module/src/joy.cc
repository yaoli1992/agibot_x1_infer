/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "joy_stick_module/joy.h"

#include <SDL2/SDL.h>
#include <unistd.h>
#include <algorithm>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace xyber_x1_infer::joy_stick_module {

Joy::Joy() {
  dev_id_ = 0;
  dev_name_ = "";
  // The user specifies the deadzone to us in the range of 0.0 to 1.0.  Later on
  // we'll convert that to the range of 0 to 32767.  Note also that negatives
  // are not allowed, as this is a +/- value.
  scaled_deadzone_ = 0.05;
  unscaled_deadzone_ = 32767.0 * 0.05;
  // According to the SDL documentation, this always returns a value between
  // -32768 and 32767.  However, we want to report a value between -1.0 and 1.0,
  // hence the "scale" dividing by 32767.  Also note that SDL returns the axis
  // with "forward" and "left" as negative.  This is opposite to the ROS
  // conventionof "forward" and "left" as positive, so we invert the axis here
  // as well.  Finally, we take into account the amount of deadzone so we truly
  // do get value between -1.0 and 1.0 (and not -deadzone to +deadzone).
  scale_ = static_cast<float>(-1.0 / (1.0 - scaled_deadzone_) / 32767.0);
  autorepeat_rate_ = 20;
  if (autorepeat_rate_ < 0.0) {
    throw std::runtime_error("Autorepeat rate must be >= 0.0");
  } else if (autorepeat_rate_ > 1000.0) {
    throw std::runtime_error("Autorepeat rate must be <= 1000.0");
  } else if (autorepeat_rate_ > 0.0) {
    autorepeat_interval_ms_ = static_cast<int>(1000.0 / autorepeat_rate_);
  } else {
    // If the autorepeat rate is set to 0, the user doesn't want us to
    // publish unless an event happens.  We still wake up every 200
    // milliseconds to check if we need to quit.
    autorepeat_interval_ms_ = 200;
  }

  sticky_buttons_ = false;
  coalesce_interval_ms_ = 1;
  if (coalesce_interval_ms_ < 0) {
    throw std::runtime_error("coalesce_interval_ms must be positive");
  }
  // Make sure to initialize publish_soon_time regardless of whether we are
  // going to use it; this ensures that we are always using the correct time
  // source.
  publish_soon_time_ = high_resolution_clock::now();

  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC) < 0) {
    throw std::runtime_error("SDL could not be initialized: " + std::string(SDL_GetError()));
  }
  // In theory we could do this with just a timer, which would simplify the code
  // a bit.  But then we couldn't react to "immediate" events, so we stick with
  // the thread.
  event_thread_ = std::thread(&Joy::eventThread, this);
}

Joy::~Joy() {
  is_runnig_.store(false);
  if (event_thread_.joinable()) {
    event_thread_.join();
  }
  if (haptic_ != nullptr) {
    SDL_HapticClose(haptic_);
  }
  if (joystick_ != nullptr) {
    SDL_JoystickClose(joystick_);
  }
  SDL_Quit();
}

void Joy::GetJoyData(JoyStruct& joy_data) {
  // while (!is_update_.load()) {
  //   std::this_thread::sleep_for(1ms);
  // }

  std::lock_guard<std::mutex> lk(joy_msg_mutex_);
  joy_data = joy_msg_;
  // is_update_.store(false);
}

float Joy::convertRawAxisValueToROS(int16_t val) {
  // SDL reports axis values between -32768 and 32767.  To make sure
  // we report out scaled value between -1.0 and 1.0, we add one to
  // the value iff it is exactly -32768.  This makes all of the math
  // below work properly.
  if (val == -32768) {
    val = -32767;
  }
  // Note that we do all of the math in double space below.  This ensures
  // that the values stay between -1.0 and 1.0.
  double double_val = static_cast<double>(val);
  // Apply the deadzone semantic here.  This allows the deadzone
  // to be "smooth".
  if (double_val > unscaled_deadzone_) {
    double_val -= unscaled_deadzone_;
  } else if (double_val < -unscaled_deadzone_) {
    double_val += unscaled_deadzone_;
  } else {
    double_val = 0.0;
  }
  return static_cast<float>(double_val * scale_);
}

bool Joy::handleJoyAxis(const SDL_Event& e) {
  bool publish = false;
  if (e.jaxis.which != joystick_instance_id_) {
    return publish;
  }
  if (e.jaxis.axis >= joy_msg_.axis.size()) {
    std::cerr << "Saw axis too large for this device, ignoring" << std::endl;
    return publish;
  }
  float last_axis_value = joy_msg_.axis.at(e.jaxis.axis);
  joy_msg_.axis.at(e.jaxis.axis) = convertRawAxisValueToROS(e.jaxis.value);
  if (last_axis_value != joy_msg_.axis.at(e.jaxis.axis)) {
    if (coalesce_interval_ms_ > 0 && !publish_soon_) {
      publish_soon_ = true;
      publish_soon_time_ = high_resolution_clock::now();
    } else {
      auto time_since_publish_soon = high_resolution_clock::now() - publish_soon_time_;
      if (time_since_publish_soon >= milliseconds(coalesce_interval_ms_)) {
        publish = true;
        publish_soon_ = false;
      }
    }
  }
  // else no change, so don't publish
  return publish;
}

bool Joy::handleJoyButtonDown(const SDL_Event& e) {
  bool publish = false;
  if (e.jbutton.which != joystick_instance_id_) {
    return publish;
  }
  if (e.jbutton.button >= joy_msg_.buttons.size()) {
    std::cerr << "Saw button too large for this device, ignoring" << std::endl;
    return publish;
  }
  if (sticky_buttons_) {
    // For sticky buttons, invert 0 -> 1 or 1 -> 0
    joy_msg_.buttons.at(e.jbutton.button) = 1 - joy_msg_.buttons.at(e.jbutton.button);
  } else {
    joy_msg_.buttons.at(e.jbutton.button) = 1;
  }
  publish = true;
  return publish;
}

bool Joy::handleJoyButtonUp(const SDL_Event& e) {
  bool publish = false;
  if (e.jbutton.which != joystick_instance_id_) {
    return publish;
  }
  if (e.jbutton.button >= joy_msg_.buttons.size()) {
    std::cerr << "Saw button too large for this device, ignoring" << std::endl;
    return publish;
  }
  if (!sticky_buttons_) {
    joy_msg_.buttons.at(e.jbutton.button) = 0;
    publish = true;
  }
  return publish;
}

bool Joy::handleJoyHatMotion(const SDL_Event& e) {
  bool publish = false;

  if (e.jhat.which != joystick_instance_id_) {
    return publish;
  }

  // The hats are the last axis in the axis list.  There are two axis per hat;
  // the first of the pair is for left (positive) and right (negative), while
  // the second of the pair is for up (positive) and down (negative).

  // Determine which pair we are based on e.jhat.hat
  int num_axes = SDL_JoystickNumAxes(joystick_);
  if (num_axes < 0) {
    std::cerr << "Failed to get axis: " << SDL_GetError() << std::endl;
    return publish;
  }
  size_t axes_start_index = num_axes + e.jhat.hat * 2;
  // Note that we check axes_start_index + 1 here to ensure that we can write to
  // either the left/right axis or the up/down axis that corresponds to this
  // hat.
  if ((axes_start_index + 1) >= joy_msg_.axis.size()) {
    std::cerr << "Saw hat too large for this device, ignoring" << std::endl;
    return publish;
  }

  if (e.jhat.value & SDL_HAT_LEFT) {
    joy_msg_.axis.at(axes_start_index) = 1.0;
  }
  if (e.jhat.value & SDL_HAT_RIGHT) {
    joy_msg_.axis.at(axes_start_index) = -1.0;
  }
  if (e.jhat.value & SDL_HAT_UP) {
    joy_msg_.axis.at(axes_start_index + 1) = 1.0;
  }
  if (e.jhat.value & SDL_HAT_DOWN) {
    joy_msg_.axis.at(axes_start_index + 1) = -1.0;
  }
  if (e.jhat.value == SDL_HAT_CENTERED) {
    joy_msg_.axis.at(axes_start_index) = 0.0;
    joy_msg_.axis.at(axes_start_index + 1) = 0.0;
  }
  publish = true;

  return publish;
}

void Joy::handleJoyDeviceAdded(const SDL_Event& e) {
  if (!dev_name_.empty()) {
    int num_joysticks = SDL_NumJoysticks();
    if (num_joysticks < 0) {
      std::cerr << "Failed to get the number of joysticks: " << SDL_GetError() << std::endl;
      return;
    }
    bool matching_device_found = false;
    for (int i = 0; i < num_joysticks; ++i) {
      const char* name = SDL_JoystickNameForIndex(i);
      if (name == nullptr) {
        std::cerr << "Could not get joystick name: " << SDL_GetError() << std::endl;
        continue;
      }
      if (std::string(name) == dev_name_) {
        // We found it!
        matching_device_found = true;
        dev_id_ = i;
        break;
      }
    }
    if (!matching_device_found) {
      std::cerr << "Could not get joystick with name " << dev_name_ << ": " << SDL_GetError()
                << std::endl;
      return;
    }
  }

  if (e.jdevice.which != dev_id_) {
    return;
  }

  joystick_ = SDL_JoystickOpen(dev_id_);
  if (joystick_ == nullptr) {
    std::cerr << "Unable to open joystick " << dev_id_ << ": " << SDL_GetError() << std::endl;
    return;
  }

  // We need to hold onto this so that we can properly remove it on a
  // remove event.
  joystick_instance_id_ = SDL_JoystickGetDeviceInstanceID(dev_id_);
  if (joystick_instance_id_ < 0) {
    std::cerr << "Failed to get instance ID for joystick: " << SDL_GetError() << std::endl;
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }

  int num_buttons = SDL_JoystickNumButtons(joystick_);
  if (num_buttons < 0) {
    std::cerr << "Failed to get number of buttons: " << SDL_GetError() << std::endl;
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  joy_msg_.buttons.resize(num_buttons);

  int num_axes = SDL_JoystickNumAxes(joystick_);
  if (num_axes < 0) {
    std::cerr << "Failed to get number of axis: " << SDL_GetError() << std::endl;
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  int num_hats = SDL_JoystickNumHats(joystick_);
  if (num_hats < 0) {
    std::cerr << "Failed to get number of hats: " << SDL_GetError() << std::endl;
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  joy_msg_.axis.resize(num_axes + num_hats * 2);

  // Get the initial state for each of the axis
  for (int i = 0; i < num_axes; ++i) {
    int16_t state;
    if (SDL_JoystickGetAxisInitialState(joystick_, i, &state)) {
      joy_msg_.axis.at(i) = convertRawAxisValueToROS(state);
    }
  }

  haptic_ = SDL_HapticOpenFromJoystick(joystick_);
  if (haptic_ != nullptr) {
    if (SDL_HapticRumbleInit(haptic_) < 0) {
      // Failed to init haptic.  Clean up haptic_.
      SDL_HapticClose(haptic_);
      haptic_ = nullptr;
    }
  } else {
    std::cout << "No haptic (rumble) available, skipping initialization" << std::endl;
  }
  std::cout << "Opened joystick: " << SDL_JoystickName(joystick_)
            << ".  deadzone: " << scaled_deadzone_ << std::endl;
}

void Joy::handleJoyDeviceRemoved(const SDL_Event& e) {
  if (e.jdevice.which != joystick_instance_id_) {
    return;
  }

  joy_msg_.buttons.resize(0);
  joy_msg_.axis.resize(0);
  if (haptic_ != nullptr) {
    SDL_HapticClose(haptic_);
    haptic_ = nullptr;
  }
  if (joystick_ != nullptr) {
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
  }
}

void Joy::eventThread() {
  auto last_publish = high_resolution_clock::now();

  while (is_runnig_) {
    bool should_publish = false;
    SDL_Event e;
    int wait_time_ms = autorepeat_interval_ms_;
    if (publish_soon_) {
      wait_time_ms = std::min(wait_time_ms, coalesce_interval_ms_);
    }
    int success = SDL_WaitEventTimeout(&e, wait_time_ms);
    std::lock_guard<std::mutex> lk(joy_msg_mutex_);
    if (success == 1) {
      // Succeeded getting an event
      if (e.type == SDL_JOYAXISMOTION) {
        should_publish = handleJoyAxis(e);
      } else if (e.type == SDL_JOYBUTTONDOWN) {
        should_publish = handleJoyButtonDown(e);
      } else if (e.type == SDL_JOYBUTTONUP) {
        should_publish = handleJoyButtonUp(e);
      } else if (e.type == SDL_JOYHATMOTION) {
        should_publish = handleJoyHatMotion(e);
      } else if (e.type == SDL_JOYDEVICEADDED) {
        handleJoyDeviceAdded(e);
      } else if (e.type == SDL_JOYDEVICEREMOVED) {
        handleJoyDeviceRemoved(e);
      } else {
        std::cout << "Unknown event type " << e.type << std::endl;
        ;
      }
    } else {
      // We didn't succeed, either because of a failure or because of a timeout.
      // If we are autorepeating and enough time has passed, set should_publish.
      auto now = high_resolution_clock::now();
      auto diff_since_last_publish = now - last_publish;
      if ((autorepeat_rate_ > 0.0 &&
           diff_since_last_publish >= milliseconds(autorepeat_interval_ms_)) ||
          publish_soon_) {
        last_publish = now;
        should_publish = true;
        publish_soon_ = false;
      }
    }

    if (joystick_ != nullptr && should_publish) {
      is_update_.store(true);
    }
  }
}
}  // namespace xyber_x1_infer::joy_stick_module
