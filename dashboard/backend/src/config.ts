/** 환경 변수는 여기 한 곳에서만 읽는다. 나머지 모듈은 이 객체를 가져다 쓴다. */
export const config = {
  port: Number(process.env.PORT ?? 5174),

  db: {
    host: process.env.DB_HOST ?? '127.0.0.1',
    port: Number(process.env.DB_PORT ?? 3306),
    user: process.env.DB_USER ?? 'valet',
    password: process.env.DB_PASSWORD ?? '',
    database: process.env.DB_NAME ?? 'valet',
  },

  /** rosbridge. 규석 VM 에 붙일 때는 ws://<VM_IP>:9090 으로 준다. */
  rosbridgeUrl: process.env.ROSBRIDGE_URL ?? 'ws://127.0.0.1:9090',
  rosReconnectMs: 3000,
} as const;
