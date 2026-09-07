import { Router } from 'express';
import { getRobotPose } from '../ros/collector.ts';
import { layout } from '../layout.ts';

export const poseRoutes = Router();

/**
 * 로봇 현재 위치. DB 를 타지 않고 메모리의 최신값을 그대로 준다.
 *
 * 차체 치수도 같이 보낸다 — 프런트가 도면에 실제 크기로 그려야
 * 통로 폭이나 주차면 여유가 눈에 들어온다.
 */
poseRoutes.get('/pose', (_req, res) => {
  const pose = getRobotPose();
  res.json({
    pose,
    stale: pose ? Date.now() - Date.parse(pose.stamp) > 3000 : true,
    robot: {
      length: layout.robot_spec?.length ?? 4.5,
      width: layout.robot_spec?.width ?? 1.9,
    },
  });
});
