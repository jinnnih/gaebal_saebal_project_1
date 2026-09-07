import { Router } from 'express';
import { layoutRoutes } from './layout.ts';
import { spotRoutes } from './spots.ts';
import { requestRoutes } from './requests.ts';
import { metricRoutes } from './metrics.ts';
import { poseRoutes } from './pose.ts';

/** 모든 라우트는 /api 아래에 붙는다. 프런트(Vite)가 이 접두어로 프록시한다. */
export const api = Router();
api.use(layoutRoutes);
api.use(spotRoutes);
api.use(requestRoutes);
api.use(metricRoutes);
api.use(poseRoutes);
