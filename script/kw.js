#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='rzyz deg';

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  let tarr=[euler[0].toFixed(6),euler[1].toFixed(6),euler[2].toFixed(6)]
  let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
  return tarr.concat(rarr).join(",");
}

protocol.delim=",";
protocol.lf="";
protocol.joints=['joint_1_s', 'joint_2_l', 'joint_3_u', 'joint_4_r', 'joint_5_b', 'joint_6_t'];
module.exports=protocol;
