import { useState } from 'react';
import ThemeDecorator from '@enact/sandstone/ThemeDecorator';
import { HashRouter as Router, Route, Routes, Link } from 'react-router-dom'; // HashRouter로 변경
import Panels from '@enact/sandstone/Panels';
import MainPanel from '../views/MainPanel'; // 메인 페이지 컴포넌트
import HarvibotPage from '../views/HarvestView'; // 하비봇 페이지 컴포넌트
import CCTVPage from '../views/cctvview'; // CCTV 페이지 컴포넌트
import HubotPage from '../views/HubotView'; // 휴봇 페이지 컴포넌트
import css from './App.module.less';

const App = (props) => {
    // 슬라이드 메뉴 상태 관리
    const [isOpen, setIsOpen] = useState(false);

    // 메뉴 열기/닫기 핸들러
    const toggleMenu = () => {
        setIsOpen(!isOpen);
    };

    return (
        <Router> {/* HashRouter로 설정 */}
            <div className={css.app}>
                {/* 메뉴 버튼 (햄버거 아이콘) */}
                <div className={css["menu-icon"]} onClick={toggleMenu}>
                    &#9776; {/* 햄버거 아이콘 */}
                </div>

                {/* 슬라이드 메뉴 */}
                <div className={`${css.sidebar} ${isOpen ? css.open : ''}`}>
                    <ul className={css["menu-items"]}>
                        {/* Link 컴포넌트로 각각의 경로 연결 */}
                        <li>
                            <Link to="/" onClick={toggleMenu}>메인</Link>
                        </li>
                        <li>
                            <Link to="/cctv" onClick={toggleMenu}>CCTV</Link>
                        </li>
                        <li>
                            <Link to="/harvibot" onClick={toggleMenu}>하비봇</Link>
                        </li>
                        <li>
                            <Link to="/hubot" onClick={toggleMenu}>휴봇</Link>
                        </li>
                    </ul>
                </div>

                {/* 페이지 전환을 관리하는 Routes 설정 */}
                <Panels {...props} style={{ backgroundColor: '#a6b28f' }}>
                    <Routes>
                        <Route path="/" element={<MainPanel />} />
                        <Route path="/cctv" element={<CCTVPage />} />
                        <Route path="/hubot" element={<HubotPage />} />
                        <Route path="/harvibot" element={<HarvibotPage />} />
                    </Routes>
                </Panels>
            </div>
        </Router>
    );
};

export default ThemeDecorator(App);
