// Auto-generated. Do not edit!

// (in-package moobot_ui.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class check_imu {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angular_vel_z = null;
      this.linear_acc_x = null;
    }
    else {
      if (initObj.hasOwnProperty('angular_vel_z')) {
        this.angular_vel_z = initObj.angular_vel_z
      }
      else {
        this.angular_vel_z = 0.0;
      }
      if (initObj.hasOwnProperty('linear_acc_x')) {
        this.linear_acc_x = initObj.linear_acc_x
      }
      else {
        this.linear_acc_x = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type check_imu
    // Serialize message field [angular_vel_z]
    bufferOffset = _serializer.float32(obj.angular_vel_z, buffer, bufferOffset);
    // Serialize message field [linear_acc_x]
    bufferOffset = _serializer.float32(obj.linear_acc_x, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type check_imu
    let len;
    let data = new check_imu(null);
    // Deserialize message field [angular_vel_z]
    data.angular_vel_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linear_acc_x]
    data.linear_acc_x = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_ui/check_imu';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b1b998330ad15a2d1903c80d31c89e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 angular_vel_z
    float32 linear_acc_x
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new check_imu(null);
    if (msg.angular_vel_z !== undefined) {
      resolved.angular_vel_z = msg.angular_vel_z;
    }
    else {
      resolved.angular_vel_z = 0.0
    }

    if (msg.linear_acc_x !== undefined) {
      resolved.linear_acc_x = msg.linear_acc_x;
    }
    else {
      resolved.linear_acc_x = 0.0
    }

    return resolved;
    }
};

module.exports = check_imu;
