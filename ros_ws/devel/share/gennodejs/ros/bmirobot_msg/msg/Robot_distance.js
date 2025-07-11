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

class Robot_distance {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.proximity = null;
      this.realdistance = null;
    }
    else {
      if (initObj.hasOwnProperty('proximity')) {
        this.proximity = initObj.proximity
      }
      else {
        this.proximity = new Array(25).fill(0);
      }
      if (initObj.hasOwnProperty('realdistance')) {
        this.realdistance = initObj.realdistance
      }
      else {
        this.realdistance = new Array(25).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Robot_distance
    // Check that the constant length array field [proximity] has the right length
    if (obj.proximity.length !== 25) {
      throw new Error('Unable to serialize array field proximity - length must be 25')
    }
    // Serialize message field [proximity]
    bufferOffset = _arraySerializer.uint16(obj.proximity, buffer, bufferOffset, 25);
    // Check that the constant length array field [realdistance] has the right length
    if (obj.realdistance.length !== 25) {
      throw new Error('Unable to serialize array field realdistance - length must be 25')
    }
    // Serialize message field [realdistance]
    bufferOffset = _arraySerializer.uint16(obj.realdistance, buffer, bufferOffset, 25);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Robot_distance
    let len;
    let data = new Robot_distance(null);
    // Deserialize message field [proximity]
    data.proximity = _arrayDeserializer.uint16(buffer, bufferOffset, 25)
    // Deserialize message field [realdistance]
    data.realdistance = _arrayDeserializer.uint16(buffer, bufferOffset, 25)
    return data;
  }

  static getMessageSize(object) {
    return 100;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_msg/Robot_distance';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '68c6629711bc8cca4c5688c4b92123ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint16[25] proximity
    uint16[25] realdistance
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Robot_distance(null);
    if (msg.proximity !== undefined) {
      resolved.proximity = msg.proximity;
    }
    else {
      resolved.proximity = new Array(25).fill(0)
    }

    if (msg.realdistance !== undefined) {
      resolved.realdistance = msg.realdistance;
    }
    else {
      resolved.realdistance = new Array(25).fill(0)
    }

    return resolved;
    }
};

module.exports = Robot_distance;
