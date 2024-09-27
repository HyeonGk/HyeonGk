import React, { useState, useEffect } from 'react';
import Calendar from 'react-calendar'; // 달력 컴포넌트
import 'react-calendar/dist/Calendar.css'; // 기본 달력 스타일
import css from './MainPanel.module.less'; // 커스터마이징 CSS 파일
import { Bar } from 'react-chartjs-2'; // 차트 컴포넌트 가져오기
import { Chart as ChartJS, BarElement, CategoryScale, LinearScale, Tooltip, Legend } from 'chart.js';


ChartJS.register(BarElement, CategoryScale, LinearScale, Tooltip, Legend);

const MainPanel = () => {
    const [currentDate, setCurrentDate] = useState(new Date());
    const [selectedDate, setSelectedDate] = useState(new Date());
    const [showNewWidgets, setShowNewWidgets] = useState(false); // 위젯 전환 상태 변수
    const [selectedCrop, setSelectedCrop] = useState(''); // 선택한 작물
    const [harvestData, setHarvestData] = useState({}); // 수확 데이터

    // 센서 데이터 변수
    const [sensorData, setSensorData] = useState({
        temperature: null,
        humidity: null,
    });

    const fetchSensorData = async () => {
        try {
            const response = await fetch('http://172.20.10.7/');
            const data = await response.json();
            setSensorData({
                temperature: data.temperature,
                humidity: data.humidity,
            });
        } catch(error) {
            console.error('Failed to fetch sensor data:', error);
        }
    };
    
    //렌더링될 때 가져오기
    useEffect(() => {
        fetchSensorData();
        const interval = setInterval(fetchSensorData, 5000);
        return () => clearInterval(interval);
    }, []);

    // 오늘 날짜를 업데이트
    useEffect(() => {
        const timer = setInterval(() => {
            setCurrentDate(new Date());
        }, 1000); // 1초마다 업데이트

        return () => clearInterval(timer);
    }, []);

    // 달력에서 날짜를 선택할 때 호출되는 함수
    const onDateChange = (date) => {
        setSelectedDate(date);
    };

    // 화살표 버튼 클릭 시 위젯 전환
    const toggleWidgets = () => {
        setShowNewWidgets(!showNewWidgets); // 새로운 위젯 표시 여부 토글
    };

    // 작물 선택 시 수확량 표시
    const handleCropChange = (event) => {
        const selected = event.target.value;
        setSelectedCrop(selected);
        // 수확 데이터 가정: 9월 23일에 각각 1개씩 수확됨
        if (selected) {
            setHarvestData({
                apple: 1,
                lemon: 1,
                orange: 1,
            });
        } else {
            setHarvestData({});
        }
    };

 // 선택한 작물에 따라 색상을 변경하는 함수
   const getFruitDetails = () => {
    switch (selectedCrop) {
        case 'apple':
            return '#ef4107'; // 빨간색
        case 'lemon':
            return '#f4d03f'; // 노란색
        case 'orange':
            return '#ffa500'; // 주황색
        default:
            return '#333'; // 기본 색상
        }
    };

    const color  = getFruitDetails();

     // 막대 그래프 데이터를 정의
     const barData = {
        labels: ['9/18', '9/22', '9/23'], // X축: 날짜
        datasets: [
            {
                label: '수확량',
                data: selectedCrop === 'apple' ? [2, 5, 3] : [], // Y축: 수확 개수
                backgroundColor: color, // 막대 색상
            },
        ],
    };

    const barOptions = {
        responsive: true,
        scales: {
            y: {
                beginAtZero: true, // Y축이 0부터 시작
            },
        },
    };

    
    return (
        <div className={css.container}>
            {/* 화면 중앙 상단에 "ELECBRO" */}
            <h1 className={css.title}>ELECBRO</h1>

            <div className={css.gridContainer}>
                {/* 첫 번째와 두 번째 위젯 */}
                {!showNewWidgets ? (
                    <>
                        <div className={css.calendar}>
                            <h2>오늘 날짜</h2>
                            {/* 날짜와 시간을 한 줄에 표시 */}
                            <p className={css.inlineDate}>
                                {currentDate.toLocaleDateString()} {currentDate.toLocaleTimeString()}
                            </p>

                            {/* 달력 위젯 */}
                            <Calendar
                                onChange={onDateChange}
                                value={selectedDate}
                                locale="ko-KR" // 한국어 로케일 설정
                                className={css.customCalendar} // 커스터마이징된 스타일
                            />
                        </div>

                        {/* 두 번째 영역 - 온도, 습도, CO2 */}
                        <div className={css.sensorData}>
                            <h2>센서 데이터</h2>
                            <div className={css.sensorGrid}>
                                <div className={css.sensorItem}>
                                    <h3>온도</h3>
                                    <p>{sensorData.temperature !== null ? `${sensorData.temperature}°C` : 'Loading...'}</p>
                                </div>
                                <div className={css.sensorItem}>
                                    <h3>습도</h3>
                                    <p>{sensorData.humidity !== null ? `${sensorData.humidity}%` : 'Loading...'}</p>
                                </div>
                                <div className={css.sensorItem}>
                                    <h3>CO2</h3>
                                    <p>400ppm</p>
                                </div>
                            </div>
                        </div>
                    </>
                ) : (
                    /* 세 번째와 네 번째 위젯 */
                    <>
                        <div className={css.newWidget}>
                            <h2>수확 작물 현황</h2>
                            {/* 커스텀 스타일 적용된 Select Box */}
                            <div className={css.selectBox}>
                                <select className={css.select} value={selectedCrop} onChange={handleCropChange}>
                                    <option value="">작물 선택</option>
                                    <option value="apple">사과</option>
                                    <option value="lemon">레몬</option>
                                    <option value="orange">오렌지</option>
                                </select>
                            
                            </div>

                            {/* 수확량 표시 */}
                            <div className={css.harvestResult} style={{ color }}>
                                {selectedCrop === 'apple' && <p>사과: {harvestData.apple}개</p>}
                                {selectedCrop === 'lemon' && <p>레몬: {harvestData.lemon}개</p>}
                                {selectedCrop === 'orange' && <p>오렌지: {harvestData.orange}개</p>}
                            </div>
                             {/* 사과가 선택된 경우에만 그래프 표시 */}
                             {selectedCrop === 'apple' && (
                                <div className={css.chartContainer}>
                                    <Bar data={barData} options={barOptions} />
                                </div>
                            )}
                        </div>

                        <div className={css.newWidget}>
                            <h2>로봇 상태</h2>
                            <p>(휴봇과 하비봇의 상태)</p>
                        </div>
                    </>
                )}

                {/* 화살표 버튼 */}
                <div className={css.arrowButton} onClick={toggleWidgets}>
                    {showNewWidgets ? '◀' : '▶'} {/* 화살표 방향 */}
                </div>
            </div>
        </div>
    );
};

export default MainPanel;
