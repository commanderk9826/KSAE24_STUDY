# KSAE24_STUDY
<h3>KSAE 전자_제어팀, 스터디 시간에 공부한 것을 자유롭게 올리는 repository 입니다.</h3>

<h2>1. 스터디 git 사용법</h2>

git clone https://github.com/commanderk9826/KSAE24_STUDY.git

cd KSAE24_STUDY

git log --oneline

git checkout -b [본인 이름]

git add .

git commit -am "Add [날짜] study"

git push -u origin [본인 이름]

<h2>2. 이후 스터디 내용 업로드할때</h2>

git pull : 본인 브랜치에서 수정이 이뤄지지 않았더라도, 다른 사람의 브랜치의 수정내용이 발생하면, 변경사항을 업데이트 해줘야 함.

git log --oneline : commit 확인 및, 브랜치 자기 이름인지 확인

git add .

git commit -m "Add [날짜] study"

*git commit -am "Add [날짜] study" : add . 와 commit 동시에 진행하는 명령어

git push

끝!

