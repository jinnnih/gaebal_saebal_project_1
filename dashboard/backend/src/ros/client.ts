/**
 * rosbridge WebSocket 연결.
 *
 * 연결·재접속·구독·발행만 맡는다. 받은 메시지를 DB 에 넣는 일은 collector.ts 가 한다.
 * Node 22+ 내장 WebSocket 을 쓰므로 의존성이 없다.
 */
import { config } from '../config.ts';
import { MSG_TYPE, POSE_MSG_TYPE, TOPIC, QOS } from './contract.ts';

/**
 * 원본 메시지를 그대로 넘긴다.
 * std_msgs/String 토픽은 `msg.data` 가 JSON 문자열이고, `/amcl_pose` 같은 표준
 * 메시지는 `msg` 자체가 객체다. 어느 쪽인지는 collector 가 토픽별로 판단한다.
 */
type Handler = (topic: string, msg: any) => void | Promise<void>;

let ws: WebSocket | null = null;
let handler: Handler = () => {};
let stopped = false;

function subscribe(topic: string, qos: object, type: string = MSG_TYPE) {
  ws?.send(JSON.stringify({ op: 'subscribe', topic, type, qos }));
}

function connect() {
  if (stopped) return;
  console.log(`[ros] 연결 시도 ${config.rosbridgeUrl}`);
  ws = new WebSocket(config.rosbridgeUrl);

  ws.onopen = () => {
    console.log('[ros] 연결됨');
    subscribe(TOPIC.spotStates, QOS.spotStates);
    subscribe(TOPIC.missionStatus, QOS.missionStatus);
    subscribe(TOPIC.robotPose, QOS.robotPose, POSE_MSG_TYPE);
    ws!.send(JSON.stringify({ op: 'advertise', topic: TOPIC.request, type: MSG_TYPE }));
  };

  ws.onmessage = async (ev) => {
    try {
      const frame = JSON.parse(String(ev.data));
      if (frame.op !== 'publish') return;
      await handler(frame.topic, frame.msg);
    } catch (e: any) {
      console.error('[ros] 메시지 처리 실패:', e?.message ?? e);
    }
  };

  ws.onclose = () => {
    if (stopped) return;
    console.log(`[ros] 연결 끊김 — ${config.rosReconnectMs / 1000}초 후 재시도`);
    setTimeout(connect, config.rosReconnectMs);
  };
  ws.onerror = () => { /* onclose 가 이어서 불린다 */ };
}

export function startRosClient(onMessage: Handler) {
  handler = onMessage;
  stopped = false;
  connect();
}

export function stopRosClient() {
  stopped = true;
  ws?.close();
  ws = null;
}

export const isConnected = () => ws?.readyState === WebSocket.OPEN;

/** 연결이 없으면 false 를 돌려준다. 호출부가 DB 에만 남길지 판단한다. */
export function publish(topic: string, body: object): boolean {
  if (!isConnected()) return false;
  ws!.send(JSON.stringify({ op: 'publish', topic, msg: { data: JSON.stringify(body) } }));
  return true;
}
