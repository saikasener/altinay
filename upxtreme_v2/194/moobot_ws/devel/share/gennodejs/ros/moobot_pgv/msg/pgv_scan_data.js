// Auto-generated. Do not edit!

// (in-package moobot_pgv.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class pgv_scan_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_pos = null;
      this.y_pos = null;
      this.orientation = null;
      this.read_barcode = null;
      this.tag_num = null;
      this.lane_detected = null;
    }
    else {
      if (initObj.hasOwnProperty('x_pos')) {
        this.x_pos = initObj.x_pos
      }
      else {
        this.x_pos = 0.0;
      }
      if (initObj.hasOwnProperty('y_pos')) {
        this.y_pos = initObj.y_pos
      }
      else {
        this.y_pos = 0.0;
      }
      if (initObj.hasOwnProperty('orientation')) {
        this.orientation = initObj.orientation
      }
      else {
        this.orientation = 0.0;
      }
      if (initObj.hasOwnProperty('read_barcode')) {
        this.read_barcode = initObj.read_barcode
      }
      else {
        this.read_barcode = false;
      }
      if (initObj.hasOwnProperty('tag_num')) {
        this.tag_num = initObj.tag_num
      }
      else {
        this.tag_num = 0;
      }
      if (initObj.hasOwnProperty('lane_detected')) {
        this.lane_detected = initObj.lane_detected
      }
      else {
        this.lane_detected = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pgv_scan_data
    // Serialize message field [x_pos]
    bufferOffset = _serializer.float32(obj.x_pos, buffer, bufferOffset);
    // Serialize message field [y_pos]
    bufferOffset = _serializer.float32(obj.y_pos, buffer, bufferOffset);
    // Serialize message field [orientation]
    bufferOffset = _serializer.float32(obj.orientation, buffer, bufferOffset);
    // Serialize message field [read_barcode]
    bufferOffset = _serializer.bool(obj.read_barcode, buffer, bufferOffset);
    // Serialize message field [tag_num]
    bufferOffset = _serializer.int32(obj.tag_num, buffer, bufferOffset);
    // Serialize message field [lane_detected]
    bufferOffset = _serializer.int32(obj.lane_detected, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pgv_scan_data
    let len;
    let data = new pgv_scan_data(null);
    // Deserialize message field [x_pos]
    data.x_pos = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_pos]
    data.y_pos = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [orientation]
    data.orientation = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [read_barcode]
    data.read_barcode = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tag_num]
    data.tag_num = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [lane_detected]
    data.lane_detected = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'moobot_pgv/pgv_scan_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fb34df9bdee5103f0f83bb286c7a7b50';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x_pos
    float32 y_pos
    float32 orientation
    bool read_barcode
    int32 tag_num
    int32 lane_detected #0 when lane_detected
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pgv_scan_data(null);
    if (msg.x_pos !== undefined) {
      resolved.x_pos = msg.x_pos;
    }
    else {
      resolved.x_pos = 0.0
    }

    if (msg.y_pos !== undefined) {
      resolved.y_pos = msg.y_pos;
    }
    else {
      resolved.y_pos = 0.0
    }

    if (msg.orientation !== undefined) {
      resolved.orientation = msg.orientation;
    }
    else {
      resolved.orientation = 0.0
    }

    if (msg.read_barcode !== undefined) {
      resolved.read_barcode = msg.read_barcode;
    }
    else {
      resolved.read_barcode = false
    }

    if (msg.tag_num !== undefined) {
      resolved.tag_num = msg.tag_num;
    }
    else {
      resolved.tag_num = 0
    }

    if (msg.lane_detected !== undefined) {
      resolved.lane_detected = msg.lane_detected;
    }
    else {
      resolved.lane_detected = 0
    }

    return resolved;
    }
};

module.exports = pgv_scan_data;
