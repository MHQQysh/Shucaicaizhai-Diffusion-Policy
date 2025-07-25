// Auto-generated. Do not edit!

// (in-package bmirobot_move.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GraspServoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.object_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('object_pose')) {
        this.object_pose = initObj.object_pose
      }
      else {
        this.object_pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraspServoRequest
    // Serialize message field [object_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.object_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraspServoRequest
    let len;
    let data = new GraspServoRequest(null);
    // Deserialize message field [object_pose]
    data.object_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bmirobot_move/GraspServoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f370b349a20e2e0029b6c926ba1b874';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose object_pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GraspServoRequest(null);
    if (msg.object_pose !== undefined) {
      resolved.object_pose = geometry_msgs.msg.Pose.Resolve(msg.object_pose)
    }
    else {
      resolved.object_pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

class GraspServoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraspServoResponse
    // Serialize message field [status]
    bufferOffset = _serializer.int32(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraspServoResponse
    let len;
    let data = new GraspServoResponse(null);
    // Deserialize message field [status]
    data.status = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'bmirobot_move/GraspServoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '86791dcf1de997ec7de5a0de7e4dcfcc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GraspServoResponse(null);
    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = {
  Request: GraspServoRequest,
  Response: GraspServoResponse,
  md5sum() { return '37b88a2b91372a258e4573b0ed517a08'; },
  datatype() { return 'bmirobot_move/GraspServo'; }
};
