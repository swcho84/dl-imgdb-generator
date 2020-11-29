# dl-imgdb-generator
## Image Database Generator for CNN
------
------
## Feat. related to Cityscape DB/XML file
- 코드에 작성한 주석 참고하기
- 주석 참고 시, waitKey(0) 부분을 잘 사용하면 one-by-one 으로 기능/산출물 점검 가능함
------
### - Feat.#1: folder 내의 image 들을 resizing
- /CityScapesDBfolder/raw 에 기록된 경로의 파일들을 VGA로 resizing 하여 /CityScapesDBfolder/cvtimg 에 저장
---
### - Feat.#2: Cityscape DB 의 semantic colored labeling result 와 polygon labeling result 에서 bbox 생성하고 xml file 만들기
- colored label 을 contour 로 만들고 이것의 bounding box 를 추출함
- json type 으로 저장된 polygon label 을 읽어와서 이것의 bounding box 를 추출함
- 이 둘의 결과를 xml file 로 생성함
- pet mix option 을 넣으면 raw pet image 에서 crop 된 pet image 를 원영상에 add 하고 bbox 정보를 추가로 추출하여 xml file 로 생성함
- params.yaml 에 선언된 parameter 를 셋팅해줘야 함
- folder path 에 주의, home folder 정보는 자동으로 읽고 있으므로 이 정보 외에 다른 정보들을 잘 입력해줘야 함
- raw pet image 에서 pet image 만 crop 하는 코드는 opencv의 grabcut 알고리듬 참고하여 코딩하거나 crop한 결과만을 임시로 가져와서 사용해야 함
---
### - Feat.#3: xml file checker
- converted VGA image 와 generated xml file 의 결과 점검하는 기능
- folder path 에 주의, home folder 정보는 자동으로 읽고 있으므로 이 정보 외에 다른 정보들을 잘 입력해줘야 함
---
### - Feat.#4: gray label converter from rgb label (for semantic segmentation)
- semantic segmentation 을 위한 color labeled image 에서 gray labeled image 로의 변환
- gray labeled image 를 사용하는 경우 이 부분에 대한 변환 해야 함
- folder path 에 주의, home folder 정보는 자동으로 읽고 있으므로 이 정보 외에 다른 정보들을 잘 입력해줘야 함
---
### - Feat.#5: yolo label converter from xml file
- xml file 로부터 yolo label 로 변환해주는 코드
- folder path 에 주의, home folder 정보는 자동으로 읽고 있으므로 이 정보 외에 다른 정보들을 잘 입력해줘야 함
---
---
## Feat. related to Kitty DB/JSON file
- 코드에 작성한 주석 참고하기
- 주석 참고 시, waitKey(0) 부분을 잘 사용하면 one-by-one 으로 기능/산출물 점검 가능함
------
### - Feat.#1: folder 내의 image 들을 resizing
- /KittyDBfolder/image 에 기록된 경로의 파일들을 VGA로 resizing 하여 /KittyDBfolder/cvtimg 에 저장
---
### - Feat.#2: Kitty DB 의 text labeling result 에서 bbox 생성하고 xml file 만들기
- text 에 line 형태로 저장된 정보를 읽어와서 label 과 bbox 생성 (yolo data 와 비슷)
- 이 결과를 xml file 로 생성함
- params.yaml 에 선언된 parameter 를 셋팅해줘야 함
- folder path 에 주의, home folder 정보는 자동으로 읽고 있으므로 이 정보 외에 다른 정보들을 잘 입력해줘야 함
---
### - Feat.#3: xml file checker
- converted VGA image 와 generated xml file 의 결과 점검하는 기능
- folder path 에 주의, home folder 정보는 자동으로 읽고 있으므로 이 정보 외에 다른 정보들을 잘 입력해줘야 함
------
------
## 실행방법
- launch file 을 목적에 맞게 선택하여 실행
- 실행 전에 params.yaml 에서 목적에 맞게 parameter 셋팅해야 함
------
------
## Parameter 설명
