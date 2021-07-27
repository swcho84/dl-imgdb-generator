import os
import xml.etree.ElementTree as ET

# targetDir = r"/home/drswchogcs/v4.0_additional_data/label"
targetDir = r"/home/drswchogcs/v4.0_5000ea/label"
num = 1

##targetDir에서 .xml파일 이름들 리스트로 가져오기
file_list = os.listdir(targetDir)
xml_list = []
for file in file_list:
  if '.xml' in file:
    xml_list.append(file)

##모든 .xml파일에 대해 수정
for xml_file in xml_list:
  target_path = targetDir + "/" + xml_file
  targetXML = open(target_path, 'rt', encoding='UTF8')

  tree = ET.parse(targetXML)

  root = tree.getroot()
  objects = root.findall("object")

  for _object in objects:
    bndbox = _object.find("bndbox")
    xmin_tag = bndbox.find("xmin")
    ymin_tag = bndbox.find("ymin")
    xmax_tag = bndbox.find("xmax")
    ymax_tag = bndbox.find("ymax")

    original_xmin = xmin_tag.text
    original_ymin = ymin_tag.text
    original_xmax = xmax_tag.text
    original_ymax = ymax_tag.text

    modified_xmin = original_xmin.replace(original_xmin, original_xmax)
    modified_ymin = original_ymin.replace(original_ymin, original_ymin)
    modified_xmax = original_xmax.replace(original_xmax, original_xmin)
    modified_ymax = original_ymax.replace(original_ymax, original_ymax)    

    xmin_tag.text = modified_xmin
    ymin_tag.text = modified_ymin
    xmax_tag.text = modified_xmax
    ymax_tag.text = modified_ymax

    tree.write(target_path)

    # if (original == "우체국조끼"):
    #   print(original)
    #   modified = original.replace(r"우체국조끼",r"postman_vest")
    #   name_tag.text = modified  #수정
    #   print(name_tag.text)
    #   tree.write(target_path)
   
  print("[" + str(num) + "]" + xml_file + "[success]")

  num += 1

# confirm data
# for xml_file in xml_list:
#   target_path = targetDir + "/" + xml_file
#   targetXML = open(target_path, 'rt', encoding='UTF8')

#   tree = ET.parse(targetXML)

#   root = tree.getroot()
#   objects = root.findall("object")

#   for _object in objects:
#     name = _object.find("name").text
#     print(name)