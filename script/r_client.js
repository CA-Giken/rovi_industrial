#!/usr/bin/env node

const net=require('net');
const EventEmitter=require('events').EventEmitter;
const ros=require('rosnodejs');
const geometry_msgs=ros.require('geometry_msgs').msg;
const sensor_msgs=ros.require('sensor_msgs').msg;
const std_msgs=ros.require('std_msgs').msg;

let Config={
  robot_ip:'111.11.1.1',
  robot_port:1111,
  protocol:'',
  update_frame_id:''
};

setImmediate(async function(){
  const rosNode=await ros.initNode('r_client');
  try{
    let co=await rosNode.getParam('/config/rsocket');
    Object.assign(Config,co);
  }
  catch(e){
  }
  const protocol=require('./'+Config.protocol+'.js');
  protocol.node(rosNode);
  const pub_js=rosNode.advertise('/joint_states',sensor_msgs.JointState);
  const pub_tf=rosNode.advertise('/update/config_tf',geometry_msgs.TransformStamped);
  const pub_conn=rosNode.advertise('/rsocket/enable',std_msgs.Bool);
  const joint=new sensor_msgs.JointState();
  joint.name=['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];
  if(protocol.hasOwnProperty('joints')) joint.name=protocol.joints;

  const client = new net.Socket();
  let wdt=null;
  setTimeout(async function(){
    console.log("robot ip "+Config.robot_port+":"+Config.robot_ip);
    client.connect(Config.robot_port,Config.robot_ip);
  },3000);
  const cstat = new std_msgs.Bool();
  setInterval(function(){
    pub_conn.publish(cstat);
  },1000);
  client.on('connect', function(data) {
    console.log('r-joint::Connected');
    cstat.data=true;
    wdt=setTimeout(function(){
      cstat.data=false;
      client.destroy();
    },3000);
  });
  let msg='';
  client.on('data',async function(data){
    if(wdt!=null){
      clearTimeout(wdt);
      wdt=setTimeout(function(){
        client.destroy();
      },3000);
    }
    let last_rb=0;
    msg+=data.toString();
    msg=msg.replace( /[\r\n]+/gm, "" );
    let idx=msg.lastIndexOf('J');
    if(idx>=0){
      let lb=msg.indexOf('(',idx);
      let rb=msg.indexOf(')',idx);
      if(lb>=0 && rb>=0){
        joint.header.seq++;
        joint.header.stamp = ros.Time.now();
        try{
          let jns=protocol.jdecode(msg.slice(lb,rb+1));
          joint.position=jns[0].slice(0,joint.name.length);
          pub_js.publish(joint);
        }
        catch(e){
          ros.log.error('r_client::joint parse error '+e);
        }
        if(last_rb<rb) last_rb=rb;
      }
    }
    idx=msg.lastIndexOf('P');
    if(idx>=0){
      let lb=msg.indexOf('(',idx);
      let rb=msg.indexOf(')',idx);
      if(lb>=0 && rb>=0){
        let tfs;
        try{
          tfs=await protocol.decode(msg.slice(lb,rb+1));
          if(tfs.length>0 && tfs[0].hasOwnProperty('translation') && Config.update_frame_id.length>0){
            let tf=new geometry_msgs.TransformStamped();
            tf.header.stamp=ros.Time.now();
            tf.header.frame_id=Config.base_frame_id;
            tf.child_frame_id=Config.update_frame_id;
            tf.transform=tfs[0];
            pub_tf.publish(tf);
          }
        }
        catch(e){
          ros.log.error('r_client::pose parse error '+e);
          return;
        }
        if(last_rb<rb) last_rb=rb;
      }
    }
    msg=msg.slice(last_rb+1);
  });
  client.on('close', function() {
  	ros.log.info('r_client::socket closed');
    cstat.data=false;
    pub_conn.publish(cstat);
    if(wdt!=null) clearTimeout(wdt);
    wdt=null;
    setTimeout(function(){
      client.connect(Config.robot_port,Config.robot_ip);
    },5000);
  });
  client.on('error', function() {
    ros.log.error('r_client::socket error');
  });
  client.on('timeout', function() {
    console.log('r_client::socket timeout');
    client.destroy();    
  });
});
