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
- LabelsCityScapesDB: Cityscape DB 의 semantic colored label 읽어와서 원하는 bbox 로 만들기 위해 필요한 정보
  - contour 로 만들기 위해 필요한 parameter (cannyThresh, morphThresh, polyDPThesh) 설정
  - semantic colored label 중에 원하는 정보만 읽어내기 위해 RGB value 설정 필요 (Cityscape DB 설명 참고)
- LabelsKittyDB: Kitty DB 의 bbox label 읽어와서 원하는 bbox 로 만들기 위해 필요한 정보 
  - 이미 bbox 형태이므로 원하는 label 만 지정하면 됨
- KittyDBConverter/feature: 기능 선택
  - 11: img file resizer
  - 22: xml file generator
  - 33: xml file checker
- KittyDBfolder: Kitty DB 를 읽어오기 위한 정보 설정
  - image: raw data folder path
  - label: label data folder path
  - cvtimg: converted result folder path (resized image folder)
  - xml_label: converted result folder path (reworked label folder)
  - imgfile_type: raw data image format
  - txtfile_type: raw data label format
  - xmlfile_type: converted result label format
  - imgfile_extension: converted result image format
  - xmlfile_extension: converted result label format
  - file_name_fwd: 파일 이름 설정 시 필요한 내용 (파일 이름 앞부분 지정)
  - file_name_num_digit: 파일 이름 설정 시 필요한 내용 (파일 개수 계산 시 자리수 지정)
  - cvtimg_width: VGA width 으로 고정
  - cvtimg_height: VGA height 으로 고정 
- CityScapesDBConverter: 기능 선택, pet mixing 시 사용되는 parameter 
  - /CityScapesDBConverter/feature: 기능선택
    - 1: img file resizer
    - 2: xml file generator
    - 3: xml file checker
    - 4: seg label converter (from RGB to label)
    - 5: txt file generator from xml file for yolo
  - use_pet_mixing: pet mixing 기능 선택여부
  - trial_pet_mixing: pet mixing 몇 장 할건지 선택
  - mix_width_ratio: mixing 시 사진 가로비율
  - mix_height_ratio: mixing 시 사진 세로비율
  - mix_inner_ratio: mixing 시 사진 위치 지정
- CityScapesDBfolder: Cityscape DB 를 읽어오기 위한 정보 설정
  - raw: raw data folder path
  - cvtimg: converted result folder path (resized image folder)
  - color_label: semantic colored label folder path
  - polygon_data: json-type polygon label folder path
  - xml_label:converted result folder path (reworked label folder)
  - pet_mix: cropped pet image folder path
  - seg_color_img: /path/to/seg/color/img
  - seg_label_img: /path/to/seg/label/img
  - pet_mix_img: converted result folder path (mixed image folder)
  - yolo_label: converted result folder path (reworked yolo label folder)
  - imgfile_type: raw data image format
  - polygonfile_type: raw data json format
  - xmlfile_type: converted result label format
  - imgfile_extension: converted result image format
  - xmlfile_extension: converted result label format
  - file_name_fwd: 파일 이름 설정 시 필요한 내용 (파일 이름 앞부분 지정)
  - file_name_num_digit: 파일 이름 설정 시 필요한 내용 (파일 개수 계산 시 자리수 지정)
  - cvtimg_width: VGA width 으로 고정
  - cvtimg_height: VGA height 으로 고정  
------
------
## Installation
- 설치 필요한 libraries: CMakeList.txt 참고 (line. 7 to line 38)
- 설치방법은 인터넷 검색을 통해 참고 (대부분 설치방법 단순함)
- OpenCV 설치 시 GPU 사용을 하려면 주의해야 함 (숙련자가 수행할 것을 추천)
- 유의사항: cmake 실행 시 -DCMAKE_INSTALL_PREFIX=/usr/local 을 추가할 것
------