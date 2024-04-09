#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='rvec';

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  euler[0]*=0.001;
  euler[1]*=0.001;
  euler[2]*=0.001;
  let tarr=[euler[0].toFixed(6),euler[1].toFixed(6),euler[2].toFixed(6)]
  let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
  return "["+tarr.concat(rarr).join(",")+"]";
}
protocol.decode=async function(msg){
  let ary=protocol.decode_(msg);
  if(ary.length==0) return [];
  ary=ary.map(function(a){
    a[0]*=1000.;
    a[1]*=1000.;
    a[2]*=1000.;
    return a;
  });
  return await this.tflib.fromEuler(ary);
}
protocol.delim="\n";
protocol.lf="\n";

module.exports=protocol;

