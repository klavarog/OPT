# Сборка OPT из исходников в Windows.

* Скачайте `mingw-get-setup.exe`: 
	* [c официального сайта](http://www.mingw.org/wiki/Getting_Started) пролистать вниз, до `1. Click on this mingw-get-setup.exe`.
	* [с sourceforge](https://sourceforge.net/projects/mingw/files/Installer/), кнопка "Download Latest Version".
* Устанавите скачанный `mingw` на диск С. Не в `Program Files` или куда-либо еще. Жмите "ОК" установка завершается и выходит диалоговое окно с настройками. Это окно закрыть.   
* Скачайте утилиту [Opt](http://509.ch/opt.7z)
	* Распакуйте ее в директорию `С:/`.
	* Можно просто скопировать архив с программой на диск `С` и правой кнопкой мыши выбрать `извлечь в текущую папку`.
* Скачайте `Notepad++` и сразу же его установите. [Ссылка для скачивания.](https://notepad-plus-plus.org/download/v7.7.html)
* Запустите консоль: Пуск >> Выполнить >> пишем `cmd.exe`.
* В консоли пишем команды:
	```
	cd C:\MinGW\bin
	mingw-get.exe update
	mingw-get.exe install mingwrt
	mingw-get.exe install w32api
	mingw-get.exe install binutils 
	mingw-get.exe install gcc 
	mingw-get.exe install g++
	mingw-get.exe install mingw32-make
	set PATH=C:\MinGW\bin;%PATH%
	cd C:\opt
	g++ -std=c++11 -O2 -DNDEBUG -DTASTENZAHL=35 -DENGLISH -static-libgcc -static-libstdc++ opt.cc -o opt
	```
	Установка программы завершена. Не закрывайте консоль! Работать с программой будем в консоли.
* Не закрывая консоли откройте `NotePad++`. В пункте меню `Кодировка` выберите `UTF-8`.
	Теперь вставьте в этот документ текст, который хотите проанализировать. 
	Сохраните файл.
	В данном примере файл под названием `meinkorpus` c выбранным расширением `txt` был сохранен в директории `C:\opt`.
* В консоли пишем команду `opt meinkorpus.txt`
	Теперь откройте "C:\opt". В списке файлов появились "meinkorpus.txt.1", "meinkorpus.txt.2", "meinkorpus.txt.3" и "meinkorpus.txt.wl" Это файлы с программным анализом вашего текста. В качестве примера здесь приведена только одна из множества функций программы. Возможности программы позволяют создавать кастомные прошивки с применением слоев
под любые запросы. Смотрите руководство в README.md  и "Anleitung.pdf". Во второй половине "Anleitung.pdf" руководство на английском языке.
