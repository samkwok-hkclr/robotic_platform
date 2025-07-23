#ifndef VACUUM_GRIPPER_STATUS_HPP__
#define VACUUM_GRIPPER_STATUS_HPP__

#pragma once

class VacuumGripperStatus
{
public:
  VacuumGripperStatus()
  {
    state = false;
    pressure = 0.0f;
    distance = 0.0f;
    temperature = 0.0f;
  }

  ~VacuumGripperStatus()
  {

  }

  // Getters
  bool get_state() const { return state; }
  float get_pressure() const { return pressure; }
  float get_distance() const { return distance; }
  float get_temperature() const { return temperature; }

  // Setters
  void set_state(bool data) { state = data; }
  void set_pressure(float data) { pressure = data; }
  void set_distance(float data) { distance = data; }
  void set_temperature(float data) { temperature = data; }

private:
  bool state;
  float pressure;
  float distance;
  float temperature;

};

#endif // VACUUM_GRIPPER_STATUS_HPP__