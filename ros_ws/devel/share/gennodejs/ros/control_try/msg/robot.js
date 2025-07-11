// Auto-generated. Do not edit!

// (in-package control_try.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class robot {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_pos = null;
      this.joint_spd = null;
      this.eff_pose = null;
      this.eff_spd = null;
      this.timestamp = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_pos')) {
        this.joint_pos = initObj.joint_pos
      }
      else {
        this.joint_pos = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('joint_spd')) {
        this.joint_spd = initObj.joint_spd
      }
      else {
        this.joint_spd = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('eff_pose')) {
        this.eff_pose = initObj.eff_pose
      }
      else {
        this.eff_pose = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('eff_spd')) {
        this.eff_spd = initObj.eff_spd
      }
      else {
        this.eff_spd = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('timestamp')) {
        this.timestamp = initObj.timestamp
      }
      else {
        this.timestamp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type robot
    // Check that the constant length array field [joint_pos] has the right length
    if (obj.joint_pos.length !== 9) {
      throw new Error('Unable to serialize array field joint_pos - length must be 9')
    }
    // Serialize message field [joint_pos]
    bufferOffset = _arraySerializer.float64(obj.joint_pos, buffer, bufferOffset, 9);
    // Check that the constant length array field [joint_spd] has the right length
    if (obj.joint_spd.length !== 9) {
      throw new Error('Unable to serialize array field joint_spd - length must be 9')
    }
    // Serialize message field [joint_spd]
    bufferOffset = _arraySerializer.float64(obj.joint_spd, buffer, bufferOffset, 9);
    // Check that the constant length array field [eff_pose] has the right length
    if (obj.eff_pose.length !== 6) {
      throw new Error('Unable to serialize array field eff_pose - length must be 6')
    }
    // Serialize message field [eff_pose]
    bufferOffset = _arraySerializer.float64(obj.eff_pose, buffer, bufferOffset, 6);
    // Check that the constant length array field [eff_spd] has the right length
    if (obj.eff_spd.length !== 6) {
      throw new Error('Unable to serialize array field eff_spd - length must be 6')
    }
    // Serialize message field [eff_spd]
    bufferOffset = _arraySerializer.float64(obj.eff_spd, buffer, bufferOffset, 6);
    // Serialize message field [timestamp]
    bufferOffset = _serializer.float64(obj.timestamp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type robot
    let len;
    let data = new robot(null);
    // Deserialize message field [joint_pos]
    data.joint_pos = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [joint_spd]
    data.joint_spd = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [eff_pose]
    data.eff_pose = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [eff_spd]
    data.eff_spd = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [timestamp]
    data.timestamp = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 248;
  }

  static datatype() {
    // Returns string type for a message object
    return 'control_try/robot';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '40052a2da386ad976e3644364a1e9279';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64[9] joint_pos
    float64[9] joint_spd
    float64[6] eff_pose
    float64[6] eff_spd
    float64 timestamp
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new robot(null);
    if (msg.joint_pos !== undefined) {
      resolved.joint_pos = msg.joint_pos;
    }
    else {
      resolved.joint_pos = new Array(9).fill(0)
    }

    if (msg.joint_spd !== undefined) {
      resolved.joint_spd = msg.joint_spd;
    }
    else {
      resolved.joint_spd = new Array(9).fill(0)
    }

    if (msg.eff_pose !== undefined) {
      resolved.eff_pose = msg.eff_pose;
    }
    else {
      resolved.eff_pose = new Array(6).fill(0)
    }

    if (msg.eff_spd !== undefined) {
      resolved.eff_spd = msg.eff_spd;
    }
    else {
      resolved.eff_spd = new Array(6).fill(0)
    }

    if (msg.timestamp !== undefined) {
      resolved.timestamp = msg.timestamp;
    }
    else {
      resolved.timestamp = 0.0
    }

    return resolved;
    }
};

module.exports = robot;
