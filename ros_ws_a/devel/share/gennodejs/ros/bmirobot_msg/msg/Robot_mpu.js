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

class Robot_mpu {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mpu_Ax = null;
      this.mpu_Ay = null;
      this.mpu_Az = null;
      this.mpu_Rx = null;
      this.mpu_Ry = null;
      this.mpu_Rz = null;
    }
    else {
      if (initObj.hasOwnProperty('mpu_Ax')) {
        this.mpu_Ax = initObj.mpu_Ax
      }
      else {
        this.mpu_Ax = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mpu_Ay')) {
        this.mpu_Ay = initObj.mpu_Ay
      }
      else {
        this.mpu_Ay = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mpu_Az')) {
        this.mpu_Az = initObj.mpu_Az
      }
      else {
        this.mpu_Az = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mpu_Rx')) {
        this.mpu_Rx = initObj.mpu_Rx
      }
      else {
        this.mpu_Rx = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mpu_Ry')) {
        this.mpu_Ry = initObj.mpu_Ry
      }
      else {
        this.mpu_Ry = new Array(8).fill(0);
      }
      if (initObj.hasOwnProperty('mpu_Rz')) {
        this.mpu_Rz = initObj.mpu_Rz
      }
      else {
        this.mpu_Rz = new Array(8).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Robot_mpu
    // Check that the constant length array field [mpu_Ax] has the right length
    if (obj.mpu_Ax.length !== 8) {
      throw new Error('Unable to serialize array field mpu_Ax - length must be 8')
    }
    // Serialize message field [mpu_Ax]
    bufferOffset = _arraySerializer.int16(obj.mpu_Ax, buffer, bufferOffset, 8);
    // Check that the constant length array field [mpu_Ay] has the right length
    if (obj.mpu_Ay.length !== 8) {
      throw new Error('Unable to serialize array field mpu_Ay - length must be 8')
    }
    // Serialize message field [mpu_Ay]
    bufferOffset = _arraySerializer.int16(obj.mpu_Ay, buffer, bufferOffset, 8);
    // Check that the constant length array field [mpu_Az] has the right length
    if (obj.mpu_Az.length !== 8) {
      throw new Error('Unable to serialize array field mpu_Az - length must be 8')
    }
    // Serialize message field [mpu_Az]
    bufferOffset = _arraySerializer.int16(obj.mpu_Az, buffer, bufferOffset, 8);
    // Check that the constant length array field [mpu_Rx] has the right length
    if (obj.mpu_Rx.length !== 8) {
      throw new Error('Unable to serialize array field mpu_Rx - length must be 8')
    }
    // Serialize message field [mpu_Rx]
    bufferOffset = _arraySerializer.int16(obj.mpu_Rx, buffer, bufferOffset, 8);
    // Check that the constant length array field [mpu_Ry] has the right length
    if (obj.mpu_Ry.length !== 8) {
      throw new Error('Unable to serialize array field mpu_Ry - length must be 8')
    }
    // Serialize message field [mpu_Ry]
    bufferOffset = _arraySerializer.int16(obj.mpu_Ry, buffer, bufferOffset, 8);
    // Check that the constant length array field [mpu_Rz] has the right length
    if (obj.mpu_Rz.length !== 8) {
      throw new Error('Unable to serialize array field mpu_Rz - length must be 8')
    }
    // Serialize message field [mpu_Rz]
    bufferOffset = _arraySerializer.int16(obj.mpu_Rz, buffer, bufferOffset, 8);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Robot_mpu
    let len;
    let data = new Robot_mpu(null);
    // Deserialize message field [mpu_Ax]
    data.mpu_Ax = _arrayDeserializer.int16(buffer, bufferOffset, 8)
    // Deserialize message field [mpu_Ay]
    data.mpu_Ay = _arrayDeserializer.int16(buffer, bufferOffset, 8)
    // Deserialize message field [mpu_Az]
    data.mpu_Az = _arrayDeserializer.int16(buffer, bufferOffset, 8)
    // Deserialize message field [mpu_Rx]
    data.mpu_Rx = _arrayDeserializer.int16(buffer, bufferOffset, 8)
    // Deserialize message field [mpu_Ry]
    data.mpu_Ry = _arrayDeserializer.int16(buffer, bufferOffset, 8)
    // Deserialize message field [mpu_Rz]
    data.mpu_Rz = _arrayDeserializer.int16(buffer, bufferOffset, 8)
    return data;
  }

  static getMessageSize(object) {
    return 96;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bmirobot_msg/Robot_mpu';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '742fca3c0a78013d1f45f4495e1ad202';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16[8] mpu_Ax
    int16[8] mpu_Ay
    int16[8] mpu_Az
    int16[8] mpu_Rx
    int16[8] mpu_Ry
    int16[8] mpu_Rz
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Robot_mpu(null);
    if (msg.mpu_Ax !== undefined) {
      resolved.mpu_Ax = msg.mpu_Ax;
    }
    else {
      resolved.mpu_Ax = new Array(8).fill(0)
    }

    if (msg.mpu_Ay !== undefined) {
      resolved.mpu_Ay = msg.mpu_Ay;
    }
    else {
      resolved.mpu_Ay = new Array(8).fill(0)
    }

    if (msg.mpu_Az !== undefined) {
      resolved.mpu_Az = msg.mpu_Az;
    }
    else {
      resolved.mpu_Az = new Array(8).fill(0)
    }

    if (msg.mpu_Rx !== undefined) {
      resolved.mpu_Rx = msg.mpu_Rx;
    }
    else {
      resolved.mpu_Rx = new Array(8).fill(0)
    }

    if (msg.mpu_Ry !== undefined) {
      resolved.mpu_Ry = msg.mpu_Ry;
    }
    else {
      resolved.mpu_Ry = new Array(8).fill(0)
    }

    if (msg.mpu_Rz !== undefined) {
      resolved.mpu_Rz = msg.mpu_Rz;
    }
    else {
      resolved.mpu_Rz = new Array(8).fill(0)
    }

    return resolved;
    }
};

module.exports = Robot_mpu;
