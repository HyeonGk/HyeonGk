import React, { useState } from 'react'; 
import css from './HubotView.module.less'; // 커스터마이징 CSS 파일
import { createToast } from '../utils/toast';

const HubotView = () => {
    const [selectedDate, setSelectedDate] = useState('');
    const [selectedFruit, setSelectedFruit] = useState('');
    const [selectedCount, setSelectedCount] = useState('');

    const handleDateChange = (event) => {
        setSelectedDate(event.target.value);
    };

    const handleFruitChange = (event) => {
        setSelectedFruit(event.target.value);
    };

    const handleCountChange = (event) => {
        setSelectedCount(event.target.value);
    };

    const handleSubmit = () => {
        if (selectedDate && selectedFruit && selectedCount) {
            const message = `등록 완료: ${selectedFruit} ${selectedCount}개 (${selectedDate})`;
            createToast(message); // toast 알림을 호출
        } else {
            createToast('모든 항목을 선택해주세요.');
        }
    };

    return (
        <div className={css.container}>
            <h1 className={css.title}>휴봇</h1>
            <div className={css.gridContainer}>
                {/* 첫 번째 위젯 */}
                <div className={css.widget}>
                    <h2>휴봇 카메라</h2>
                    <p>분류 카메라</p>
                </div>

                {/* 두 번째 위젯 */}
                <div className={css.widget}>
                    <h2>수확 작물 등록</h2>

                    <div className={css.formGroup}>
                        <label className={css.formLabel}>날짜:</label>
                        <input type="date" value={selectedDate} onChange={handleDateChange} className={css.inputBox} />
                    </div>

                    <div className={css.formGroup}>
                        <label className={css.formLabel}>과일:</label>
                        <select value={selectedFruit} onChange={handleFruitChange} className={css.selectBox}>
                            <option value="">선택하세요</option>
                            <option value="사과">사과</option>
                            <option value="레몬">레몬</option>
                            <option value="오렌지">오렌지</option>
                        </select>
                    </div>

                    <div className={css.formGroup}>
                        <label className={css.formLabel}>개수:</label>
                        <input type="number" value={selectedCount} onChange={handleCountChange} min="1" className={css.inputBox} />
                    </div>

                    <button onClick={handleSubmit} className={css.submitButton}>등록</button>
                </div>
            </div>
        </div>
    );
};

export default HubotView;
