🚗 IVS Simulation Project
Matlab/Simulink & CarMaker 기반 차량 통합 시뮬레이션

📚 개요
이 프로젝트는 Matlab/Simulink와 CarMaker를 활용하여 차량 동역학 및 통합 알고리즘을 시뮬레이션하기 위해 설계되었습니다.

src_cm4sl 폴더 내 generic_IVS 모듈을 중심으로 구성되어 있으며, 다양한 시뮬레이션 환경과 Vehicle Model을 쉽게 적용할 수 있도록 Generic 구조를 채택했습니다.

🏗️ 프로젝트 구조
bash
복사
편집
.
├── src_cm4sl/
│   └── generic_IVS/
│       ├── controllers/    # 제어 알고리즘
│       ├── models/         # 차량/센서 모델
│       ├── scripts/        # 시뮬레이션 스크립트
│       └── data/           # 설정 및 파라미터
├── README.md
└── .gitignore
⚙ 개발 환경
Matlab/Simulink : R2022b 이상 권장

CarMaker : 11.x (또는 호환 버전)

OS : Windows 10/11 (64-bit)

Git : 2.30 이상

🚀 주요 특징
✅ Simulink Vehicle Dynamics 및 Control Algorithm

✅ CarMaker TestRun 연동 시뮬레이션

✅ Generic IVS 모듈로 손쉬운 커스터마이징

✅ HIL/SIL 환경 대응

✅ 자동 실행 스크립트 제공
