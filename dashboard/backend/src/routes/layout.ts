import { Router } from 'express';
import { layoutJson, layoutSource } from '../layout.ts';

export const layoutRoutes = Router();

/** 정적 레이아웃. DB 와 무관하므로 MySQL 이 없어도 200 을 준다. */
layoutRoutes.get('/layout', (_req, res) => {
  res.type('application/json').set('x-layout-source', layoutSource).send(layoutJson);
});
