import { BrowserRouter, NavLink, Route, Routes } from 'react-router-dom';
import ConsoleView from './views/ConsoleView.tsx';
import AppView from './views/AppView.tsx';
import CarView from './views/CarView.tsx';

/**
 * 같은 발렛파킹 기능을 세 창구로 노출한다.
 *
 *   /       관제 콘솔 — 주차장 운영자. 전체 현황·큐·지표
 *   /app    사용자 앱 — 운전자 폰. 차 밖에서 요청·호출
 *   /car    차 안 화면 — 하차 전 인수인계, 복귀 안내
 *
 * 자동화 수준은 셋 다 같다(무개입 AVP). 조작 창구만 다르다.
 */
export default function App() {
  return (
    <BrowserRouter>
      <nav className="surface-switch">
        <NavLink to="/" end>관제</NavLink>
        <NavLink to="/app">사용자 앱</NavLink>
        <NavLink to="/car">차 안</NavLink>
      </nav>
      <Routes>
        <Route path="/" element={<ConsoleView />} />
        <Route path="/app" element={<AppView />} />
        <Route path="/car" element={<CarView />} />
      </Routes>
    </BrowserRouter>
  );
}
