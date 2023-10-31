// Auto-generated. Do not edit!

// (in-package pf_pgv100.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class pgv_dir_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dir_command = null;
    }
    else {
      if (initObj.hasOwnProperty('dir_command')) {
        this.dir_command = initObj.dir_command
      }
      else {
        this.dir_command = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pgv_dir_msg
    // Serialize message field [dir_command]
    bufferOffset = _serializer.uint8(obj.dir_command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pgv_dir_msg
    let len;
    let data = new pgv_dir_msg(null);
    // Deserialize message field [dir_command]
    data.dir_command = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pf_pgv100/pgv_dir_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be420e78d04d79ee4d8fd23e20966af0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 dir_command
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pgv_dir_msg(null);
    if (msg.dir_command !== undefined) {
      resolved.dir_command = msg.dir_command;
    }
    else {
      resolved.dir_command = 0
    }

    return resolved;
    }
};

module.exports = pgv_dir_msg;
