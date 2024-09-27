import kind from '@enact/core/kind';

const MainView = kind({
    name: 'MainView',
    render: () => (
        <div>
            <h1>메인 페이지</h1>
            <p>여기서는 메인 페이지의 내용을 표시합니다.</p>
        </div>
    )
});

export default MainView;
