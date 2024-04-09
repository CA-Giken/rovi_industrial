#!/usr/bin/env node

const tflib=require('./tflib');
tflib.option='rzyx rad';

const protocol=require('./protocol');
protocol.tflib=tflib;
protocol.encode=async function(tf){
  let vecs=await this.tflib.toEuler(tf);
  let euler=vecs[0];
  let tarr=[euler[0].toFixed(3),euler[1].toFixed(3),euler[2].toFixed(3)]
  let rarr=[euler[3].toFixed(6),euler[4].toFixed(6),euler[5].toFixed(6)]
  return "("+tarr.concat(rarr).join(",")+")";
}

protocol.delim="\n";
protocol.lf="\r\n";

module.exports=protocol;
