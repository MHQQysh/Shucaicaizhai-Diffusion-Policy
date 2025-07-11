// Auto-generated. Do not edit!

// (in-package bmirobot_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class motorv2_fd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mt_position = null;
      this.mt_speed = null;
      this.mt_pwm = null;
      this.mt_current = null;
    }
    else {
      if (initObj.hasOwnProperty('mt_position')) {
        this.mt_position = initObj.mt_position
      }
      else {
        this.mt_position = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mt_speed')) {
        this.mt_speed = initObj.mt_speed
      }
      else {
        this.mt_speed = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mt_pwm')) {
        this.mt_pwm = initObj.mt_pwm
      }
      else {
        this.mt_pwm = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mt_current')) {
        this.mt_current = initObj.mt_current
      }
      else {
        this.mt_current = new Array(8).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motorv2_fd
    // Check that the constant length array field [mt_position] has the right length
    if (obj.mt_position.length !== 8) {
      throw new Error('Unable to serialize array field mt_position - length must be 8')
    }
    // Serialize message field [mt_position]
    bufferOffset = _arraySerializer.int32(obj.mt_position, buffer, bufferOffset, 8);
    // Check that the constant length array field [mt_speed] has the right length
    if (obj.mt_speed.length !== 8) {
      throw new Error('Unable to serialize array field mt_speed - length must be 8')
    }
    // Serialize message field [mt_speed]
    bufferOffset = _arraySerializer.int32(obj.mt_speed, buffer, bufferOffset, 8);
    // Check that the constant length array field [mt_pwm] has the right length
    if (obj.mt_pwm.length !== 8) {
      throw new Error('Unable to serialize array field mt_pwm - length must be 8')
    }
    // Serialize message field [mt_pwm]
    bufferOffset = _arraySerializer.int16(obj.mt_pwm, buffer, bufferOffset, 8);
    // Check that the constant length array field [mt_current] has the right length
    if (obj.mt_current.length !== 8) {
      throw new Error('Unable to serialize array field mt_current - length must be 8')
    }
    // Serialize message field [mt_current]
    bufferOffset = _arraySerializer.int16(obj.mt_current, buffer, bufferOffset, 8);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motorv2_fd
    let len;
    let data = new motorv2_fd(null);
    // Deserialize message field [mt_position]
    data.mt_position = _arrayDeserializer.int32(buffer, bufferOffset, 8)
    // Deserialize message field [mt_speed]
    data.mt_speed = _arrayDeserializer.int32(buffer, bufferOffset, 8)
    // Deserialize message field [mt_pwm]
    data.mt_pwm = _arrayDeserializer.int16(buffer, bufferOffset, 8)
    // Deserialize message field [mt_current]
    data.mt_current = _arrayDeserializer.int16(buffer, bufferOffset, 8)
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_msg/motorv2_fd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a5d41159e774c193e38fa281a354be65';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[8] mt_position
    int32[8] mt_speed
    int16[8] mt_pwm
    int16[8] mt_current
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motorv2_fd(null);
    if (msg.mt_position !== undefined) {
      resolved.mt_position = msg.mt_position;
    }
    else {
      resolved.mt_position = new Array(8).fill(0)
    }

    if (msg.mt_speed !== undefined) {
      resolved.mt_speed = msg.mt_speed;
    }
    else {
      resolved.mt_speed = new Array(8).fill(0)
    }

    if (msg.mt_pwm !== undefined) {
      resolved.mt_pwm = msg.mt_pwm;
    }
    else {
      resolved.mt_pwm = new Array(8).fill(0)
    }

    if (msg.mt_current !== undefined) {
      resolved.mt_current = msg.mt_current;
    }
    else {
      resolved.mt_current = new Array(8).fill(0)
    }

    return resolved;
    }
};

module.exports = motorv2_fd;
