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

class motorv2_ctr {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mt_operation_mode = null;
      this.mt_position_goal = null;
      this.mt_speed_goal = null;
      this.mt_current_goal = null;
      this.mt_pwm_goal = null;
    }
    else {
      if (initObj.hasOwnProperty('mt_operation_mode')) {
        this.mt_operation_mode = initObj.mt_operation_mode
      }
      else {
        this.mt_operation_mode = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mt_position_goal')) {
        this.mt_position_goal = initObj.mt_position_goal
      }
      else {
        this.mt_position_goal = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mt_speed_goal')) {
        this.mt_speed_goal = initObj.mt_speed_goal
      }
      else {
        this.mt_speed_goal = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mt_current_goal')) {
        this.mt_current_goal = initObj.mt_current_goal
      }
      else {
        this.mt_current_goal = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mt_pwm_goal')) {
        this.mt_pwm_goal = initObj.mt_pwm_goal
      }
      else {
        this.mt_pwm_goal = new Array(8).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motorv2_ctr
    // Check that the constant length array field [mt_operation_mode] has the right length
    if (obj.mt_operation_mode.length !== 8) {
      throw new Error('Unable to serialize array field mt_operation_mode - length must be 8')
    }
    // Serialize message field [mt_operation_mode]
    bufferOffset = _arraySerializer.int8(obj.mt_operation_mode, buffer, bufferOffset, 8);
    // Check that the constant length array field [mt_position_goal] has the right length
    if (obj.mt_position_goal.length !== 8) {
      throw new Error('Unable to serialize array field mt_position_goal - length must be 8')
    }
    // Serialize message field [mt_position_goal]
    bufferOffset = _arraySerializer.int32(obj.mt_position_goal, buffer, bufferOffset, 8);
    // Check that the constant length array field [mt_speed_goal] has the right length
    if (obj.mt_speed_goal.length !== 8) {
      throw new Error('Unable to serialize array field mt_speed_goal - length must be 8')
    }
    // Serialize message field [mt_speed_goal]
    bufferOffset = _arraySerializer.int32(obj.mt_speed_goal, buffer, bufferOffset, 8);
    // Check that the constant length array field [mt_current_goal] has the right length
    if (obj.mt_current_goal.length !== 8) {
      throw new Error('Unable to serialize array field mt_current_goal - length must be 8')
    }
    // Serialize message field [mt_current_goal]
    bufferOffset = _arraySerializer.int32(obj.mt_current_goal, buffer, bufferOffset, 8);
    // Check that the constant length array field [mt_pwm_goal] has the right length
    if (obj.mt_pwm_goal.length !== 8) {
      throw new Error('Unable to serialize array field mt_pwm_goal - length must be 8')
    }
    // Serialize message field [mt_pwm_goal]
    bufferOffset = _arraySerializer.int32(obj.mt_pwm_goal, buffer, bufferOffset, 8);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motorv2_ctr
    let len;
    let data = new motorv2_ctr(null);
    // Deserialize message field [mt_operation_mode]
    data.mt_operation_mode = _arrayDeserializer.int8(buffer, bufferOffset, 8)
    // Deserialize message field [mt_position_goal]
    data.mt_position_goal = _arrayDeserializer.int32(buffer, bufferOffset, 8)
    // Deserialize message field [mt_speed_goal]
    data.mt_speed_goal = _arrayDeserializer.int32(buffer, bufferOffset, 8)
    // Deserialize message field [mt_current_goal]
    data.mt_current_goal = _arrayDeserializer.int32(buffer, bufferOffset, 8)
    // Deserialize message field [mt_pwm_goal]
    data.mt_pwm_goal = _arrayDeserializer.int32(buffer, bufferOffset, 8)
    return data;
  }

  static getMessageSize(object) {
    return 136;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_msg/motorv2_ctr';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28290c9afcbb7e417ea5e1d8b11ed1b4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8[8] mt_operation_mode
    int32[8] mt_position_goal
    int32[8] mt_speed_goal
    int32[8] mt_current_goal
    int32[8] mt_pwm_goal
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motorv2_ctr(null);
    if (msg.mt_operation_mode !== undefined) {
      resolved.mt_operation_mode = msg.mt_operation_mode;
    }
    else {
      resolved.mt_operation_mode = new Array(8).fill(0)
    }

    if (msg.mt_position_goal !== undefined) {
      resolved.mt_position_goal = msg.mt_position_goal;
    }
    else {
      resolved.mt_position_goal = new Array(8).fill(0)
    }

    if (msg.mt_speed_goal !== undefined) {
      resolved.mt_speed_goal = msg.mt_speed_goal;
    }
    else {
      resolved.mt_speed_goal = new Array(8).fill(0)
    }

    if (msg.mt_current_goal !== undefined) {
      resolved.mt_current_goal = msg.mt_current_goal;
    }
    else {
      resolved.mt_current_goal = new Array(8).fill(0)
    }

    if (msg.mt_pwm_goal !== undefined) {
      resolved.mt_pwm_goal = msg.mt_pwm_goal;
    }
    else {
      resolved.mt_pwm_goal = new Array(8).fill(0)
    }

    return resolved;
    }
};

module.exports = motorv2_ctr;
