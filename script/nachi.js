#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='rzyx deg';

const protocol=require('./protocol');
protocol.tflib=tflib;

function zeropad(N){
    return (N.toFixed(3)+'00000').slice(10);
}
protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  return euler.map(function(N){
        return (N.toFixed(3)+'0000000').slice(0,10);
  }).join(",");
}
protocol.decode=async function(msg){
  let ary=protocol.decode_(msg);
  if(ary.length==0) return [];
//  ary=ary.map(function(a){
//    let z=a[3];
//    a[3]=a[5];
//    a[5]=z;
//    return a;
//  });
  return await this.tflib.fromEuler(ary);
}
protocol.jdecode=function(msg){
  let jnts=this.decode_(msg);
  return jnts.map((jnt)=>{
    if(jnt.length<this.joints.length) throw new Error("Joint number not enough");
    return jnt.map(function(j){
      return j*0.0001;
    });
  }).slice(0,this.joints.length);
}

protocol.delim=",";
protocol.lf="\n";
protocol.autoclose=true;
protocol.joints=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'];

module.exports=protocol;
