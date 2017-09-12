Модификация бортового компьютера Mitsubishi Carisma.
-------------------------
  Многих владельцев автомобиля Mitsubishi Cаrisma без штатной магнитолы интересует возможность доработки штатного БК и задействие нижней строки дисплея для вывода какой-либо полезной информации. Мне удалось этого добиться путем добавления в штатную конструкцию микроконтроллера на плате arduino. Так как поступает много запросов поделиться исходниками или внести дополнения, я решил поделиться своим проектом здесь, чтобы желающие могли дополнить данный проект, либо просто его повторить.

  Все что будет описано **относится к автомобилю без климат контроля с номером БК MR975046**. На данный момент известно что есть еще одна версия БК а/м без климат контроля - MR381288. Разработка прошивки под нее, а также под БК а/м с "климатом" пока под вопросом
Также известно, что в БК Space Star используется другой контроллер дисплея, то есть доработку без серьезного изменения схемы и кода не сделать.

Ну и, естественно, **все что вы делаете, вы делаете на свой страх и риск**! Будьте аккуратны и внимательны, если в чем то не разобрались, то лучше этого вообще не делать, чтобы ненароком не остаться без БК вообще.

  URL:

[Инструкция по самостоятельной доработке](https://github.com/miheych/carisma_bk/blob/master/Docs/Hardware%20guide.md)

[Исходный код](https://github.com/miheych/carisma_bk/blob/master/Source/bkRomV9.23_b017.ino)

[Скомпилированные прошивки](https://github.com/miheych/carisma_bk/tree/master/Hex)

[Обсуждение на сайте carisma-club.su](http://carisma-club.su/index.php?showtopic=2685)
