// Auto-generated. Do not edit!

// (in-package pf_pgv100.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class pgv_scan_data {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.angle = null;
      this.x_pos = null;
      this.y_pos = null;
      this.direction = null;
      this.color_lane_count = null;
      this.no_color_lane = null;
      this.no_pos = null;
      this.tag_detected = null;
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
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
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = '';
      }
      if (initObj.hasOwnProperty('color_lane_count')) {
        this.color_lane_count = initObj.color_lane_count
      }
      else {
        this.color_lane_count = 0;
      }
      if (initObj.hasOwnProperty('no_color_lane')) {
        this.no_color_lane = initObj.no_color_lane
      }
      else {
        this.no_color_lane = 0;
      }
      if (initObj.hasOwnProperty('no_pos')) {
        this.no_pos = initObj.no_pos
      }
      else {
        this.no_pos = 0;
      }
      if (initObj.hasOwnProperty('tag_detected')) {
        this.tag_detected = initObj.tag_detected
      }
      else {
        this.tag_detected = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pgv_scan_data
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [angle]
    bufferOffset = _serializer.float32(obj.angle, buffer, bufferOffset);
    // Serialize message field [x_pos]
    bufferOffset = _serializer.float32(obj.x_pos, buffer, bufferOffset);
    // Serialize message field [y_pos]
    bufferOffset = _serializer.float32(obj.y_pos, buffer, bufferOffset);
    // Serialize message field [direction]
    bufferOffset = _serializer.string(obj.direction, buffer, bufferOffset);
    // Serialize message field [color_lane_count]
    bufferOffset = _serializer.uint8(obj.color_lane_count, buffer, bufferOffset);
    // Serialize message field [no_color_lane]
    bufferOffset = _serializer.uint8(obj.no_color_lane, buffer, bufferOffset);
    // Serialize message field [no_pos]
    bufferOffset = _serializer.uint8(obj.no_pos, buffer, bufferOffset);
    // Serialize message field [tag_detected]
    bufferOffset = _serializer.uint8(obj.tag_detected, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.uint8(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pgv_scan_data
    let len;
    let data = new pgv_scan_data(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [angle]
    data.angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_pos]
    data.x_pos = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_pos]
    data.y_pos = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [direction]
    data.direction = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [color_lane_count]
    data.color_lane_count = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [no_color_lane]
    data.no_color_lane = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [no_pos]
    data.no_pos = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [tag_detected]
    data.tag_detected = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.direction);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pf_pgv100/pgv_scan_data';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7d3a20e54967aa3cdd07e4fe0a6961b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 angle
    float32 x_pos
    float32 y_pos
    string direction
    uint8 color_lane_count
    uint8 no_color_lane
    uint8 no_pos
    uint8 tag_detected
    uint8 id
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pgv_scan_data(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

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

    if (msg.direction !== undefined) {
      resolved.direction = msg.direction;
    }
    else {
      resolved.direction = ''
    }

    if (msg.color_lane_count !== undefined) {
      resolved.color_lane_count = msg.color_lane_count;
    }
    else {
      resolved.color_lane_count = 0
    }

    if (msg.no_color_lane !== undefined) {
      resolved.no_color_lane = msg.no_color_lane;
    }
    else {
      resolved.no_color_lane = 0
    }

    if (msg.no_pos !== undefined) {
      resolved.no_pos = msg.no_pos;
    }
    else {
      resolved.no_pos = 0
    }

    if (msg.tag_detected !== undefined) {
      resolved.tag_detected = msg.tag_detected;
    }
    else {
      resolved.tag_detected = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

module.exports = pgv_scan_data;
