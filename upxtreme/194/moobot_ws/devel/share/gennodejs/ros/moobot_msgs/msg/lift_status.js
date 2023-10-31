// Auto-generated. Do not edit!

// (in-package moobot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class lift_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.up_lift = null;
      this.down_lift = null;
    }
    else {
      if (initObj.hasOwnProperty('up_lift')) {
        this.up_lift = initObj.up_lift
      }
      else {
        this.up_lift = false;
      }
      if (initObj.hasOwnProperty('down_lift')) {
        this.down_lift = initObj.down_lift
      }
      else {
        this.down_lift = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type lift_status
    // Serialize message field [up_lift]
    bufferOffset = _serializer.bool(obj.up_lift, buffer, bufferOffset);
    // Serialize message field [down_lift]
    bufferOffset = _serializer.bool(obj.down_lift, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type lift_status
    let len;
    let data = new lift_status(null);
    // Deserialize message field [up_lift]
    data.up_lift = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [down_lift]
    data.down_lift = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_msgs/lift_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88d1b4abfd45857169d18e9ca11502ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool up_lift
    bool down_lift
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new lift_status(null);
    if (msg.up_lift !== undefined) {
      resolved.up_lift = msg.up_lift;
    }
    else {
      resolved.up_lift = false
    }

    if (msg.down_lift !== undefined) {
      resolved.down_lift = msg.down_lift;
    }
    else {
      resolved.down_lift = false
    }

    return resolved;
    }
};

module.exports = lift_status;
