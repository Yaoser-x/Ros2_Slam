"""Exercise the actual SDK parser/checksum bodies with controlled read chunks."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("source")
parser.add_argument("output")
args = parser.parse_args()
source = Path(args.source).read_text(encoding="utf-8")
out = Path(args.output)
out.mkdir(exist_ok=False)


def body(signature):
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


prefix = r"""
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
using namespace std;
using result_t = int;
const int RESULT_OK=0, RESULT_FAIL=-1, RESULT_TIMEOUT=-2;
const int NoError=0, BlockError=1;
const int PH1=0xAA, PH2=0x55, PH3=0x66;
const int TRI_PACKMAXNODES=80, TRI_PACKHEADSIZE=10, STAMPPACKSIZE=8;
const int LIDAR_RESP_SYNCBIT=1, LIDAR_RESP_CHECKBIT=1;
const int SDK_UNIT64=64, SDK_SIZE1K=1024, TIMEOUT_2S=2000;
const int NODE_QUAL0=0, NODE_QUAL8=8, NODE_QUAL10=10, NODE_QUAL16=16;
#define IS_OK(v) ((v)==RESULT_OK)
#define error(...) ++errors
#define debug(...) ((void)0)
#define info(...) ((void)0)
#define warn(...) ((void)0)
struct __attribute__((packed)) stamp_package { uint8_t head[3]; uint32_t stamp; uint8_t reserved; };
struct Pack { vector<uint8_t> data; uint64_t stamp;
  Pack(const vector<uint8_t>& d,uint64_t s):data(d),stamp(s) {} };
class YDlidarDriver {
public:
  vector<vector<uint8_t>> chunks;
  size_t next=0;
  vector<uint8_t> m_data;
  vector<Pack> m_datas;
  uint32_t now=0, m_zeroTime=0;
  uint64_t stamp=0;
  int m_BlockRevSize=0, m_driverErrno=NoError;
  bool m_Debug=false, m_AutoIntensity=true;
  int m_csCount=0,m_intensityBit=NODE_QUAL0,errors=0,completed=0;
  vector<vector<uint8_t>> checked;
  uint32_t getms() { return now++; }
  vector<uint8_t> readAll(uint32_t) {
    if(next>=chunks.size()) return {};
    return chunks[next++];
  }
  void checkBlockStatus(uint8_t) {}
  void setDriverError(int x) { m_driverErrno=x; }
  bool parsePoints() { completed++;m_datas.clear();return true; }
  bool calcCheckSum(const vector<uint8_t>&);
  result_t parseData(uint32_t timeout);
  int getPointBytes();
};
"""
suffix = r"""
vector<uint8_t> packet(uint16_t distance,bool bad=false,uint8_t count=1) {
  vector<uint8_t> p={0xAA,0x55,0,count,1,0,3,0,0,0};
  for(int i=0;i<count;i++) {p.push_back(uint8_t(distance+i));p.push_back(uint8_t((distance+i)>>8));}
  uint16_t crc=0;
  for(size_t i=0;i<p.size();i+=2) if(i!=8) crc ^= p[i]|uint16_t(p[i+1]<<8);
  if(bad) crc^=0x4000;
  p[8]=crc;p[9]=crc>>8;return p;
}
int main() {
  auto good=packet(1000), bad=packet(1004,true);
  int failures=0,cases=0;
  auto check=[&](const string& name,const vector<vector<uint8_t>>& chunks,int errors) {
    YDlidarDriver d;d.chunks=chunks;
    auto result=d.parseData(1000);
    bool ok=result==RESULT_OK && d.errors==errors && d.m_intensityBit==0 && d.m_data.empty();
    cases++;
    cout<<(ok?"PASS ":"FAIL ")<<name<<" result="<<result
        <<" checksum_errors="<<d.errors<<" intensity="<<d.m_intensityBit<<endl;
    if(!ok) failures++;
  };
  check("whole_valid",{good},0);
  check("bad_then_valid_separate_reads",{bad,good},1);
  vector<uint8_t> both=bad;both.insert(both.end(),good.begin(),good.end());
  check("bad_and_valid_coalesced",{both},1);
  for(size_t split=1;split<good.size();split++) {
    vector<uint8_t> first(good.begin(),good.begin()+split),rest(good.begin()+split,good.end());
    check("fragmented_valid_"+to_string(split),{first,rest},0);
    vector<uint8_t> prior=bad;prior.insert(prior.end(),first.begin(),first.end());
    check("bad_plus_fragmented_valid_"+to_string(split),{prior,rest},1);
  }
  vector<vector<uint8_t>> one_byte;
  for(auto b:good) one_byte.push_back({b});
  check("one_byte_reads",one_byte,0);
  auto large=packet(2000,false,40);
  for(size_t split=1;split<large.size();split++) {
    vector<uint8_t> first(large.begin(),large.begin()+split),rest(large.begin()+split,large.end());
    check("fragmented_40_points_"+to_string(split),{first,rest},0);
    vector<uint8_t> prior=bad;prior.insert(prior.end(),first.begin(),first.end());
    check("bad_plus_fragmented_40_points_"+to_string(split),{prior,rest},1);
  }
  vector<uint8_t> stamp={0xAA,0x66,0,0xD2,0x04,0,0,0};
  for(size_t i=0;i<stamp.size();i++) if(i!=2) stamp[2]^=stamp[i];
  auto stamped=stamp;stamped.insert(stamped.end(),good.begin(),good.end());
  for(size_t split=1;split<stamp.size();split++) {
    YDlidarDriver d;
    d.chunks={vector<uint8_t>(stamped.begin(),stamped.begin()+split),
              vector<uint8_t>(stamped.begin()+split,stamped.end())};
    bool ok=d.parseData(1000)==RESULT_OK && d.errors==0 && d.stamp==1234000000ULL;
    cases++;
    cout<<(ok?"PASS ":"FAIL ")<<"fragmented_timestamp_"<<split<<endl;
    if(!ok) failures++;
  }
  cout<<"CASES="<<cases<<" FAILURES="<<failures<<endl;
  return failures?1:0;
}
"""
code = (
    prefix
    + "\n"
    + "\n".join(
        body(x)
        for x in [
            "result_t YDlidarDriver::parseData(",
            "int YDlidarDriver::getPointBytes()",
            "bool YDlidarDriver::calcCheckSum(",
        ]
    )
    + "\n"
    + suffix
)
(out / "parser_test.cpp").write_text(code, encoding="utf-8")
compile_result = subprocess.run(
    [
        "g++",
        "-std=c++14",
        "-O0",
        "-g",
        str(out / "parser_test.cpp"),
        "-o",
        str(out / "parser_test"),
    ],
    capture_output=True,
    text=True,
)
(out / "compile.log").write_text(compile_result.stdout + compile_result.stderr)
assert compile_result.returncode == 0, compile_result.stderr
result = subprocess.run([str(out / "parser_test")], capture_output=True, text=True, timeout=10)
(out / "run.log").write_text(result.stdout + result.stderr)
(out / "result.json").write_text(
    json.dumps(
        {
            "source_sha256": hashlib.sha256(source.encode()).hexdigest(),
            "exit_code": result.returncode,
            "output": result.stdout,
        },
        indent=2,
    )
)
print(result.stdout)
raise SystemExit(result.returncode)
