#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='sxyz deg';

const protocol=require('./protocol');
protocol.tflib=tflib;

protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  let tarr=[Math.floor(euler[0]*1000),Math.floor(euler[1]*1000),Math.floor(euler[2]*1000)]
  let rarr=[Math.floor(euler[3]*10000),Math.floor(euler[4]*10000),Math.floor(euler[5]*10000)]
  return tarr.concat(rarr).join(",");
}
protocol.decode=async function(msg){
  let ary=protocol.decode_(msg);
  if(ary.length==0) return [];
  ary=ary.map(function(a){
    a[0]*=0.001;
    a[1]*=0.001;
    a[2]*=0.001;
    a[3]*=0.0001;
    a[4]*=0.0001;
    a[5]*=0.0001;
    return a;
  });
  return await this.tflib.fromEuler(ary);
}
protocol.jdecode=function(msg){
  let jnts=this.decode_(msg);
  return jnts.map((jnt)=>{
    if(jnt.length<this.joints.length) throw new Error("Joint number not enough");
    return jnt.map(function(j){
      return j*0.0001*Math.PI/180;
    });
  }).slice(0,this.joints.length);
}

protocol.delim=",";
protocol.lf="\n";
protocol.joints=['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];

module.exports=protocol;
