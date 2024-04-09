#!/usr/bin/env node

const net=require('net');
const EventEmitter=require('events').EventEmitter;
const ros=require('rosnodejs');
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;

let Config={
  robot_ip:'111.11.1.1',
  robot_port:1111
};

setImmediate(async function(){
  const emitter=new EventEmitter();
  const rosNode=await ros.initNode('rjoint');
  try{
    let co=await rosNode.getParam('/config/rsocket');
    Object.assign(Config,co);
  }
  catch(e){
  }
  const pub_js=rosNode.advertise('/joint_states',sensor_msgs.JointState);
  const pub_conn=rosNode.advertise('/rjoint/stat',std_msgs.Bool);
  const joint=new sensor_msgs.JointState();
  joint.name=['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];

  const client = new net.Socket();
  const cstat=new std_msgs.Bool();
  client.connect(Config.robot_port,Config.robot_ip);
  console.log("IP "+Config.robot_port+" "+Config.robot_ip);
  client.setTimeout(3000);
  client.on('connect', function(data) {
    console.log('r-joint::Connected');
    let f=new std_msgs.Bool();
    cstat.data=true;
    pub_conn.publish(cstat);
  });
  client.on('data', function(data) {
    let jnt;
    try{
      jnt=JSON.parse('['+data+']');
    }
    catch(e){
      console.log('r-joint::parse error');
      return;
    }
    joint.header.seq++;
    joint.header.stamp = ros.Time.now();
    if(jnt.length<6) return;
    jnt=jnt.slice(0,6);
    joint.position=jnt.map(function(e){
        return e*Math.PI*0.0001/180;
    });
    pub_js.publish(joint);
  });
  client.on('close', function() {
	console.log('r-joint::closed');
    cstat.data=false;
    pub_conn.publish(cstat);
    setTimeout(function(){
      client.connect(Config.robot_port,Config.robot_ip);
    },5000);
  });
  client.on('error', function() {
    console.log('r-joint::error');
  });
  client.on('timeout', function() {
    console.log('r-joint::timeout');
    client.destroy();    
  });
});
