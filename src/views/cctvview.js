import kind from '@enact/core/kind';
import { Component } from 'react';
import css from './CCTVView.module.less';

const CCTVView = kind({
    name: 'CCTVView',

    render: ({ streamSrc, onCaptureClick, onStopClick, onStartClick }) => (
        <div className={css.container}>
            <h1 className={css.title}>CCTV</h1>

            {/* 실시간 CCTV 화면 */}
            <div className={css.widget}>
                {streamSrc ? (
                    <img
                        id="cctvStream"
                        src={streamSrc}
                        alt="ESP32-CAM Stream"
                        style={{ width: '100%', height: 'auto' }}
                    />
                ) : (
                    <p>현재 프레임 고정됨</p>
                )}
            </div>

            {/* 하단 영역 - 시작, 중지, 캡처 버튼 */}
            <div className={css.footer}>
                <button className={css.startButton} onClick={onStartClick}>
                    Start
                </button>
                <button className={css.stopButton} onClick={onStopClick}>
                    Stop
                </button>
                <button className={css.captureButton} onClick={onCaptureClick}>
                    Capture
                </button>
            </div>
        </div>
    )
});

class CCTVContainer extends Component {
    constructor(props) {
        super(props);
        this.state = {
            streamSrc: "http://172.20.10.2:81/stream", // ESP32-CAM의 스트림 URL
            lastFrame: null
        };
    }

    // 시작 버튼 기능 (스트리밍 시작)
    handleStartClick = () => {
        this.setState({ streamSrc: "http://172.20.10.2:81/stream" });
    };

    // 중지 버튼 기능 (스트리밍 중지, 현재 프레임 유지)
    handleStopClick = () => {
        const imgElement = document.getElementById('cctvStream');
        if (imgElement) {
            // 현재 프레임을 저장하고 스트림 중지
            this.setState({ lastFrame: imgElement.src, streamSrc: null });
        }
    };

    // 캡처 기능 (현재 프레임을 저장)
    handleCaptureClick = () => {
        const imgElement = document.getElementById('cctvStream');
        if (imgElement) {
            const a = document.createElement('a');
            a.href = imgElement.src;
            a.download = 'capture.jpg'; // 저장할 파일명
            a.click();
        }
    };

    render() {
        const { streamSrc, lastFrame } = this.state;
        return (
            <CCTVView
                streamSrc={streamSrc || lastFrame} // 스트리밍이 중지되면 마지막 프레임 표시
                onStartClick={this.handleStartClick}
                onStopClick={this.handleStopClick}
                onCaptureClick={this.handleCaptureClick}
            />
        );
    }
}

export default CCTVContainer;
