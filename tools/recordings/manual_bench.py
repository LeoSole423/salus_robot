import asyncio,json,time,argparse
import websockets

parser = argparse.ArgumentParser(description="Operator-requested stationary camera capture bench; never starts a mission.")
parser.add_argument("--url", required=True)
parser.add_argument("--stationary-capture-authorized", action="store_true", required=True)
args = parser.parse_args()

async def main():
 async with websockets.connect(args.url) as ws:
  async def request(op, **fields):
   token=op+str(time.monotonic())
   await ws.send(json.dumps({'op':op,'client_req_id':token,**fields}))
   while True:
    value=json.loads(await asyncio.wait_for(ws.recv(),12))
    if value.get('client_req_id')==token: return value
  state=await request('recordings_status')
  assert state['ok'] and not state['manual'] and not state['mission'] and not state['mission_stale'],state
  denied=await request('recordings_start')
  assert not denied['ok'],denied
  print('Read status available; manual start correctly denied while locked.',flush=True)
  unlocked=await request('set_control_lock',locked=False)
  assert unlocked['ok'],unlocked
  try:
   result=await request('recordings_start');assert result['ok'] and result['manual'],result
   for _ in range(12):
    await request('control_heartbeat')
    await asyncio.sleep(.5)
   state=await request('recordings_status')
   print('Manual capture via WebSocket',json.dumps(state['cameras']),flush=True)
   assert all(c['recording'] for c in state['cameras']),state
  finally:
   result=await request('recordings_stop');assert result['ok'] and not result['manual'],result
   await request('set_control_lock',locked=True)
  print('Manual capture stopped and control lock restored.',flush=True)
asyncio.run(main())
