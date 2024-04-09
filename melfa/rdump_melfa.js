#!/usr/bin/env node

const dgram = require('dgram');
const Rad2Deg = 180/Math.PI;

const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const geometry_msgs=ros.require('geometry_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;

let Config={
  robot_ip:'111.11.1.1',
  robot_port:[12000,12001] //[send,receive]
};

setImmediate(async function(){
  const rosNode=await ros.initNode('rdump');
  try{
    let co=await rosNode.getParam('/config/rsocket');
    Object.assign(Config,co);
  }
  catch(e){
  }
  const protocol=require('../script/melfa.js');
  protocol.node(rosNode);
  const pub_js=rosNode.advertise('/joint_states',sensor_msgs.JointState);
  const pub_tf=rosNode.advertise('/update/config_tf',geometry_msgs.TransformStamped);

  const joint=new sensor_msgs.JointState();
  joint.name=['joint1','joint2','joint3','joint4','joint5','joint6'];

  const client = dgram.createSocket({type:'udp4'});
  client.bind(Config.robot_port[1]);
  let wdt=false;

  client.on('error', (err) => {
    console.log(`udp error:\n${err.stack}`);
    client.close();
  });
  client.on('message', async (msg, remote) => {
    let vp=new DataView(msg.buffer);
    joint.header.seq++;
    joint.header.stamp = ros.Time.now();
    for(let i=0;i<joint.name.length;i++){
      joint.position[i]=vp.getFloat32(8+4*i,true);
    }
    pub_js.publish(joint);

    if(Config.update_frame_id.length>0){//will update tool0_controller
      let pos="(";
      for(let i=0;i<6;i++){
        pos+= i<3? vp.getFloat32(68+4*i,true):vp.getFloat32(68+4*i,true)*Rad2Deg;
        pos+= i<5? ",":")";
      }
      let tf=new geometry_msgs.TransformStamped();
      tf.header.stamp=ros.Time.now();
      tf.header.frame_id=Config.base_frame_id;
      tf.child_frame_id=Config.update_frame_id;
      let tfs=await protocol.decode(pos);
      if(tfs.length>0){
        tf.transform=tfs[0];
        pub_tf.publish(tf);
      }
    }
    wdt=true;
  });

  setInterval(function(){//start realtime monitor
    if(!wdt){
      const buffer = Uint8Array.from({length:196});
      const view = new DataView(buffer.buffer);
      view.setUint16(0,1,true); //Start realtime monitor
      view.setUint16(4,2,true); //Monitor#1 as Joint
      view.setUint16(64,1,true); //Monitor#2 as Tool0
      client.send(buffer,0,buffer.length,Config.robot_port[0],Config.robot_ip);
      console.log("send queue");
    }
    wdt=false;
  },2000);
});
