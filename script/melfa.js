#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='rzyx deg';

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  let tarr=[euler[0].toFixed(3),euler[1].toFixed(3),euler[2].toFixed(3)]
  let rarr=[euler[5].toFixed(3),euler[4].toFixed(3),euler[3].toFixed(3)]
  return "("+tarr.concat(rarr).join(",")+")(7,0)";
}
protocol.decode=async function(msg){
  let ary=protocol.decode_(msg);
  if(ary.length==0) return [];
  ary=ary.map(function(a){
    let z=a[3];
    a[3]=a[5];
    a[5]=z;
    return a;
  });
  return await this.tflib.fromEuler(ary);
}
protocol.jdecode=function(msg){
  let jnts=this.decode_(msg);
  return jnts.map((jnt)=>{
    if(jnt.length<this.joints.length) throw new Error("Joint number not enough");
    return jnt.map(function(j){
      return j*Math.PI/180;
    });
  }).slice(0,this.joints.length);
}

protocol.delim="\n";
protocol.lf="\n";
protocol.joints=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];

module.exports=protocol;

