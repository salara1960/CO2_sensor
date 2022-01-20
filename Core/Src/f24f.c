//------------------------------------------------------------------------------
// This is Open source software. You can place this code on your site, but don't
// forget a link to my YouTube-channel: https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// ��� ����������� ����������� ���������������� ��������. �� ������ ���������
// ��� �� ����� �����, �� �� �������� ������� ������ �� ��� YouTube-�����
// "����������� � ���������" https://www.youtube.com/channel/UChButpZaL5kUUl_zTyIDFkQ
// �����: �������� ������ / Nadyrshin Ruslan
//------------------------------------------------------------------------------
#include "font.h"
#include "f24f.h"


// ������� �������� ������ �����
// ������ 2 ����� ������� - ������ � ������ (��� ������������ ������� - ���������)
const uint8_t f24f_table[f24f_NOFCHARS][48 + 2] = {
	// 0x20
	{
	9,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________
	}
	// 0x21 !
	,{
	4,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	________,________,
	________,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x22 "
	,{
	11,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	_XXXX__X,XXX_____,
	_XXXX__X,XXX_____,
	_XXXX__X,XXX_____,
	__XXX___,XXX_____,
	__XXX___,XXX_____,
	__XXX___,XXX_____,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x23 #
	,{
	13,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	___XXX__,XXX_____,
	___XXX__,XXX_____,
	___XXX__,XXX_____,
	___XXX__,XXX_____,
	_XXXXXXX,X_______,
	_XXXXXXX,X_______,
	_XXXXXXX,X_______,
	_XXXXXXX,X_______,
	__XXX__X,XX______,
	__XXX__X,XX______,
	__XXX__X,XX______,
	_XXXXXXX,________,
	_XXXXXXX,________,
	_XXXXXXX,________,
	_XXXXXXX,________,
	XXX__XXX,________,
	XXX__XXX,________,
	XXX__XXX,________,
	XXX__XXX,________,
	________,________,
	________,________,
	________,________}
	// 0x24 $
	,{
	10,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	____XX__,________,
	____XX__,________,
	___XXXX_,________,
	__XXXXXX,________,
	_XXXXXXX,X_______,
	XX__XX_X,XX______,
	XX__XX__,________,
	XXX_XX__,________,
	_XXXXX__,________,
	__XXXXX_,________,
	___XXXXX,________,
	XX__XXXX,X_______,
	XX__XXXX,XX______,
	_XX_XX_X,XX______,
	_XXXXXXX,X_______,
	__XXXXXX,________,
	___XXXX_,________,
	____XX__,________,
	____XX__,________,
	________,________,
	________,________}
	// 0x25 %
	,{
	16,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	__XXX___,____XX__,
	_XXXXX__,___XX___,
	XXX__XXX,__XX____,
	XXX__XXX,_XX_____,
	XXX__XXX,XX______,
	_XXXXX_X,X_______,
	__XXX__X,X_______,
	______XX,_XXX____,
	_____XX_,XXXXX___,
	____XX_X,XX__XXX_,
	___XX__X,XX__XXX_,
	__XX___X,XX__XXX_,
	_XX_____,XXXXX___,
	XX______,_XXX____,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x26
	,{
	12,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	___XXXXX,________,
	__XXXXXX,X_______,
	__XXX__X,XX______,
	_XXX____,XXX_____,
	_XXX____,XXX_____,
	_XXX____,XXX_____,
	__XXXX_X,XX______,
	__XXXXXX,X_______,
	_XXXXXXX,X_______,
	_XX____X,XX______,
	XXX_____,XXX_____,
	XXX_____,XXX_____,
	_XXX____,_XXX____,
	_XXX____,__XXX___,
	_XXX____,_XXX____,
	__XXXXXX,XX______,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x27 '
	,{
	4,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	_XXXX___,________,
	_XXXX___,________,
	_XXXX___,________,
	__XXX___,________,
	__XXX___,________,
	__XXX___,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x28
	,{
	5,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	__XXX___,________,
	__XXX___,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	XXX_____,________,
	XXX_____,________,
	XXX_____,________,
	XXX_____,________,
	XXX_____,________,
	XXX_____,________,
	XXX_____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	__XXX___,________,
	__XXX___,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x29
	,{
	5,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	XXX_____,________,
	XXX_____,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	__XXX___,________,
	__XXX___,________,
	__XXX___,________,
	__XXX___,________,
	__XXX___,________,
	__XXX___,________,
	__XXX___,________,
	_XXX____,________,
	_XXX____,________,
	_XXX____,________,
	XXX_____,________,
	XXX_____,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x2A
	,{
	10,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	_XX_XX_X,X_______,
	__XXXXXX,________,
	XXXXXXXX,XX______,
	__XXXXXX,________,
	_XX_XX_X,X_______,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x2B +
	,{
	12,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	____XXXX,________,
	____XXXX,________,
	____XXXX,________,
	____XXXX,________,
	XXXXXXXX,XXXX____,
	XXXXXXXX,XXXX____,
	XXXXXXXX,XXXX____,
	____XXXX,________,
	____XXXX,________,
	____XXXX,________,
	____XXXX,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x2C ,
	,{
	5,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	_XXXX___,________,
	_XXXXX__,________,
	_XXXXX__,________,
	__XXXX__,________,
	__XXXX__,________,
	__XXX___,________,
	_XXX____,________}
	// 0x2D  -
	,{
	6,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	__XXXXXX,________,
	__XXXXXX,________,
	__XXXXXX,________,
	__XXXXXX,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x2E  .
	,{
	7,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	__XXX___,________,
	_XXXXX__,________,
	_XXXXX__,________,
	__XXX___,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x2F  /
	,{
	7,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	____XXXX,________,
	____XXXX,________,
	____XXXX,________,
	____XXXX,________,
	___XXXX_,________,
	___XXXX_,________,
	___XXXX_,________,
	___XXXX_,________,
	__XXXX__,________,
	__XXXX__,________,
	__XXXX__,________,
	__XXXX__,________,
	_XXXX___,________,
	_XXXX___,________,
	_XXXX___,________,
	_XXXX___,________,
	________,________,
	________,________,
	________,________,
	________,________}
  // 0x30
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ____XXXX,________,
    __XXXXXX,XX______,
    __XXX__X,XX______,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    __XXX__X,XX______,
    __XXXXXX,XX______,
    ____XXXX,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x31
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ______XX,X_______,
    ______XX,X_______,
    _____XXX,X_______,
    ___XXXXX,X_______,
    __XXXXXX,X_______,
    __XXX_XX,X_______,
    __X___XX,X_______,
    ______XX,X_______,
    ______XX,X_______,
    ______XX,X_______,
    ______XX,X_______,
    ______XX,X_______,
    ______XX,X_______,
    ______XX,X_______,
    ______XX,X_______,
	______XX,X_______,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x32
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ___XXXXX,X_______,
    __XXXXXX,XX______,
    _XXXX__X,XXX_____,
    _XXX____,XXX_____,
    ________,XXX_____,
    ________,XXX_____,
    _______X,XX______,
    ______XX,X_______,
    _____XXX,________,
    ____XXX_,________,
    ___XXX__,________,
    __XXX___,________,
	_XXX____,________,
    _XXX____,________,
    _XXXXXXX,XXX_____,
    _XXXXXXX,XXX_____,
	________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x33
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ___XXXXX,________,
    __XXXXXX,X_______,
    _XXX___X,XX______,
    _XXX___X,XX______,
    _______X,XX______,
    ______XX,XX______,
    ____XXXX,X_______,
    ____XXXX,X_______,
    _______X,XX______,
    ________,XXX_____,
	________,XXX_____,
    ________,XXX_____,
    _XXX____,XXX_____,
    _XXXX__X,XX______,
    __XXXXXX,XX______,
    ___XXXXX,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x34
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ______XX,X_______,
    _____XXX,X_______,
    _____XXX,X_______,
    ____XXXX,X_______,
    ____XXXX,X_______,
    ___XX_XX,X_______,
    __XX__XX,X_______,
    __XX__XX,X_______,
    _XX___XX,X_______,
    XX____XX,X_______,
    XXXXXXXX,XXX_____,
    XXXXXXXX,XXX_____,
    ______XX,X_______,
    ______XX,X_______,
    ______XX,X_______,
	______XX,X_______,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x35
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ___XXXXX,XX______,
    ___XXXXX,XX______,
    __XXX___,________,
    __XXX___,________,
    __XXX___,________,
    __XXXXXX,X_______,
    _XXXXXXX,XX______,
    _XXX___X,XXX_____,
    ________,XXX_____,
    ________,XXX_____,
	________,XXX_____,
    ________,XXX_____,
    _XXX____,XXX_____,
    _XXXX__X,XX______,
    __XXXXXX,XX______,
    ___XXXXX,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x36
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ____XXXX,X_______,
    __XXXXXX,XX______,
    __XXX__X,XXX_____,
    _XXX____,X_______,
    _XXX____,________,
    _XXX_XXX,X_______,
    _XXXXXXX,XX______,
    _XXXX__X,XX______,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
	_XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    __XXX__X,XX______,
    __XXXXXX,XX______,
    ____XXXX,X_______,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x37
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    _XXXXXXX,XXX_____,
    _XXXXXXX,XXX_____,
    ________,XXX_____,
    _______X,XX______,
    ______XX,X_______,
    ______XX,X_______,
    _____XXX,________,
    _____XXX,________,
    ____XXX_,________,
    ____XXX_,________,
    ____XXX_,________,
    ___XXX__,________,
    ___XXX__,________,
    ___XXX__,________,
    ___XXX__,________,
	___XXX__,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x38
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ___XXXXX,X_______,
    __XXXXXX,XX______,
    _XXXX__X,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    __XXX__X,XX______,
    ___XXXXX,X_______,
    ___XXXXX,X_______,
    __XXX__X,XX______,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
	_XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXXX__X,XXX_____,
    __XXXXXX,XX______,
    ___XXXXX,X_______,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x39
  ,{
    12,
    f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ____XXXX,________,
    __XXXXXX,XX______,
    __XXX__X,XX______,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    _XXX____,XXX_____,
    __XXX__X,XXX_____,
    __XXXXXX,XXX_____,
    ___XXXX_,XXX_____,
    ________,XXX_____,
	________,XXX_____,
    __XX____,XXX_____,
    _XXXX__X,XX______,
    __XXXXXX,XX______,
    ___XXXXX,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________}
  // 0x3A  :
  ,{
    4,
    f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	_XXXX___,________,
	_XXXX___,________,
	_XXXX___,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	_XXXX___,________,
	_XXXX___,________,
	_XXXX___,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x3B
	,{
	4,
	f24_FLOAT_HEIGHT,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    _XXXX___,________,
    _XXXX___,________,
    _XXXX___,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    ________,________,
    _XXXX___,________,
    _XXXX___,________,
    _XXXX___,________,
    ___XX___,________,
    ___XX___,________,
    ___X____,________,
    ________,________,
    ________,________,
    ________,________}
	// 0x3C <
	,{
	11,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,XXX_____,
	_______X,XX______,
	_____XXX,________,
	___XXXX_,________,
	_XXXX___,________,
	XXX_____,________,
	_XXXX___,________,
	___XXXX_,________,
	_____XXX,________,
	_______X,XX______,
	________,XXX_____,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x3D  =
	,{
	8,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	XXXXXXXX,________,
	XXXXXXXX,________,
	XXXXXXXX,________,
	________,________,
	________,________,
	________,________,
	XXXXXXXX,________,
	XXXXXXXX,________,
	XXXXXXXX,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x3E >
	,{
	9,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	XXX_____,________,
	_XXX____,________,
	__XXX___,________,
	___XXX__,________,
	____XXX_,________,
	_____XXX,________,
	______XX,X_______,
	_____XXX,________,
	____XXX_,________,
	___XXX__,________,
	__XXX___,________,
	_XXX____,________,
	XXX_____,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________,
	________,________}
	// 0x3F ?
	,{
	11,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	________,________,
	___XXXXX,________,
	__XXXXXX,X_______,
	_XXX___X,XX______,
	XXX_____,XXX_____,
	XXX_____,XXX_____,
	________,XXX_____,
	_______X,XX______,
	______XX,X_______,
	_____XXX,________,
	____XXX_,________,
	____XXX_,________,
	____XXX_,________,
	____XXX_,________,
	________,________,
	________,________,
	____XXX_,________,
	____XXX_,________,
	________,________,
	________,________,
	________,________}
	// 0x40
	,{
	16,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	_______X,X_______,
	_____XXX,XXX_____,
	___XXXXX,XXXXX___,
	__XXX___,___XXX__,
	_XXX__XX,X_XXXX__,
	_XX_XXXX,XXXX_XX_,
	XXX_XX__,_XXX_XX_,
	XX_XX___,_XX__XX_,
	XX_XX___,_XX__XX_,
	XX_XX___,_XX__XX_,
	XX_XX___,_XX__XX_,
	XX_XX___,_XX__XX_,
	XX_XX___,XXX_XX__,
	XX_XXXXX,XXXXX___,
	_XX_XXXX,_XXX____,
	_XXX____,_____XX_,
	__XXX___,___XXX__,
	___XXXXX,XXXXX___,
	_____XXX,XXX_____,
	_______X,X_______,
	________,________,
	________,________}
	// 0x41 A
	,{
	14,
	f24_FLOAT_HEIGHT,
	________,________,
	________,________,
	________,________,
	_____XXX,X_______,
	____XXXX,XX______,
	___XXX__,XXX_____,
	___XXX__,XXX_____,
	__XXX___,_XXX____,
	_XXX____,__XXX___,
	_XXX____,__XXX___,
	_XXX____,__XXX___,
	_XXX____,__XXX___,
	_XXX____,__XXX___,
	_XXXXXXX,XXXXX___,
	XXXXXXXX,XXXXXX__,
	XXX_____,___XXX__,
	XXX_____,___XXX__,
	XXX_____,___XXX__,
	XXX_____,___XXX__,
	XXX_____,___XXX__,
	XXX_____,___XXX__,
	________,________,
	________,________,
	________,________}
	// 0x42  B
	,{
	14,
	f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   ________,________,
   _XXXXXXX,X_______,
   _XXXXXXX,XX______,
   _XXXXXXX,XXX_____,
   _XXX____,_XXX____,
   _XXX____,_XXX____,
   _XXX____,_XXX____,
   _XXX____,XXX_____,
   _XXXXXXX,XX______,
   _XXXXXXX,X_______,
   _XXXXXXX,XX______,
   _XXX____,XXX_____,
   _XXX____,_XXX____,
   _XXX____,_XXX____,
   _XXX____,_XXX____,
   _XXXXXXX,XXX_____,
   _XXXXXXX,XX______,
   _XXXXXXX,X_______,
   ________,________,
   ________,________,
   ________,________}
// 0x43  C
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   ___XXXXX,X_______,
   _XXXXXXX,XXX_____,
   _XXX____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,_XXX____,
   _XXX____,_XXX____,
   _XXXXXXX,XXX_____,
   ___XXXXX,X_______,
   ________,________,
   ________,________,
   ________,________}
// 0x44  D
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXXXXXXX,________,
   XXXXXXXX,X_______,
   XXXXXXXX,XX______,
   XXX_____,XXX_____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,XXX_____,
   XXXXXXXX,XX______,
   XXXXXXXX,X_______,
   XXXXXXXX,________,
   ________,________,
   ________,________,
   ________,________}
// 0x45  E
 ,{
   12,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXXXXXXX,XX______,
   XXXXXXXX,XX______,
   XXXXXXXX,XX______,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXXXXXXX,XX______,
   XXXXXXXX,XX______,
   XXXXXXXX,XX______,
   XXXXXXXX,XX______,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXXXXXXX,XX______,
   XXXXXXXX,XX______,
   XXXXXXXX,XX______,
   ________,________,
   ________,________,
   ________,________}
// 0x46  F
 ,{
   12,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXXXXXXX,X_______,
   XXXXXXXX,X_______,
   XXXXXXXX,X_______,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXXXXXXX,X_______,
   XXXXXXXX,X_______,
   XXXXXXXX,X_______,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   ________,________,
   ________,________,
   ________,________}
// 0x47  G
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   ___XXXXX,XX______,
   __XXXXXX,XXX_____,
   _XXX____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX____X,XXXX____,
   XXX____X,XXXX____,
   XXX____X,XXXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   _XXX____,XXX_____,
   _XXXXXXX,XX______,
   ___XXXXX,X_______,
   ________,________,
   ________,________,
   ________,________}
// 0x48  H
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXXXXXXX,XXX_____,
   XXXXXXXX,XXX_____,
   XXXXXXXX,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   ________,________,
   ________,________,
   ________,________}
// 0x49  I
 ,{
   6,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXXXX___,________,
   XXXXX___,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   _XXX____,________,
   XXXXX___,________,
   XXXXX___,________,
   ________,________,
   ________,________,
   ________,________}
// 0x4A  J
 ,{
   12,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   _____XXX,X_______,
   _____XXX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   XXX___XX,X_______,
   XXX___XX,X_______,
   XXX___XX,X_______,
   XXX___XX,X_______,
   _XXXXXXX,________,
   __XXXXX_,________,
   ________,________,
   ________,________,
   ________,________}
// 0x4B  K
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,_XXX____,
   XXX_____,XXX_____,
   XXX____X,XX______,
   XXX___XX,X_______,
   XXX___XX,X_______,
   XXX__XXX,________,
   XXX_XXX_,________,
   XXXXXX__,________,
   XXXXXXX_,________,
   XXXX_XXX,________,
   XXX___XX,X_______,
   XXX___XX,X_______,
   XXX____X,XX______,
   XXX____X,XX______,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   ________,________,
   ________,________,
   ________,________}
// 0x4C  L
 ,{
   10,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXXXXXXX,________,
   XXXXXXXX,________,
   XXXXXXXX,________,
   ________,________,
   ________,________,
   ________,________}
// 0x4D  M
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,__XXX___,
   XXXX____,_XXXX___,
   XXXX____,_XXXX___,
   XXXXX___,XXXXX___,
   XXXXX___,XXXXX___,
   XXX_X___,X_XXX___,
   XXX_X___,X_XXX___,
   XXX_XX_X,X_XXX___,
   XXX_XX_X,X_XXX___,
   XXX_XX_X,X_XXX___,
   XXX_XXXX,X_XXX___,
   XXX__XXX,__XXX___,
   XXX___X_,__XXX___,
   XXX___X_,__XXX___,
   XXX_____,__XXX___,
   XXX_____,__XXX___,
   XXX_____,__XXX___,
   XXX_____,__XXX___,
   ________,________,
   ________,________,
   ________,________}
// 0x4E  N
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXXX____,_XXX____,
   XXXXX___,_XXX____,
   XXXXX___,_XXX____,
   XX_XXX__,_XXX____,
   XX_XXX__,_XXX____,
   XX__XXX_,_XXX____,
   XX__XXX_,_XXX____,
   XX___XXX,_XXX____,
   XX____XX,XXXX____,
   XX_____X,XXXX____,
   XX______,XXXX____,
   XX______,_XXX____,
   XX______,_XXX____,
   XX______,_XXX____,
   XX______,_XXX____,
   ________,________,
   ________,________,
   ________,________}
// 0x4F  O
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   ___XXXXX,X_______,
   __XXXXXX,XX______,
   _XXX____,XXX_____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   _XXX____,XXX_____,
   __XXXXXX,XX______,
   ___XXXXX,X_______,
   ________,________,
   ________,________,
   ________,________}
// 0x50  P
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXXXXXX_,________,
   XXXXXXXX,________,
   XXX___XX,X_______,
   XXX____X,XX______,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX____X,XX______,
   XXX___XX,X_______,
   XXXXXXXX,________,
   XXXXXXX_,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   XXX_____,________,
   ________,________,
   ________,________,
   ________,________}
// 0x51  Q
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   ___XXXXX,X_______,
   __XXXXXX,XX______,
   _XXXXXXX,XXX_____,
   _XXX____,XXX_____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX__XX_,XXX_____,
   _XXX__XX,XXX_____,
   _XXXX__X,XX______,
   __XXXXXX,X_______,
   ___XXXXX,XX______,
   ________,_XX_____,
   ________,__XX____,
   ________,________,
   ________,________}
// 0x52  R
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXXXXXXX,________,
   XXXXXXXX,X_______,
   XXXXXXXX,XX______,
   XXX_____,XXX_____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,XXX_____,
   XXXXXXXX,XX______,
   XXXXXXXX,________,
   XXXXXXXX,________,
   XXX___XX,X_______,
   XXX____X,XX______,
   XXX____X,XX______,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   ________,________,
   ________,________,
   ________,________}
// 0x53  S
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   __XXXXXX,X_______,
   _XXXXXXX,XX______,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,________,
   _XXX____,________,
   __XXXX__,________,
   ___XXXX_,________,
   _____XXX,X_______,
   _______X,XXX_____,
   ________,XXXX____,
   XXX_____,XXXX____,
   XXX_____,XXXX____,
   XXX____X,XXX_____,
   _XXXXXXX,XX______,
   __XXXXXX,X_______,
   ________,________,
   ________,________,
   ________,________}
// 0x54  T
 ,{
   12,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXXXXXXX,XXX_____,
   XXXXXXXX,XXX_____,
   XXXXXXXX,XXX_____,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ________,________,
   ________,________,
   ________,________}
// 0x55  U
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   _XXXX__X,XXX_____,
   __XXXXXX,XX______,
   ___XXXXX,X_______,
   ________,________,
   ________,________,
   ________,________}
// 0x56  Y
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,__XXX___,
   XXX_____,__XXX___,
   XXX_____,__XXX___,
   _XXX____,_XXX____,
   _XXX____,_XXX____,
   _XXX____,_XXX____,
   __XXX___,XXX_____,
   __XXX___,XXX_____,
   __XXX___,XXX_____,
   ___XXX_X,XX______,
   ___XXX_X,XX______,
   ___XXX_X,XX______,
   ____XXXX,X_______,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   ________,________,
   ________,________,
   ________,________}
// 0x57  W
 ,{
   16,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,____XXX_,
   XXX____X,____XXX_,
   XXX___XX,X___XXX_,
   XXX___XX,X___XXX_,
   XXX___XX,X___XXX_,
   _XXX__XX,X__XXX__,
   _XXX_XX_,XX_XXX__,
   _XXX_XX_,XX_XXX__,
   _XXX_XX_,XX_XXX__,
   __XX_XX_,XX_XX___,
   __XX_XX_,XX_XX___,
   __XX_XX_,XX_XX___,
   __XX_XX_,XX_XX___,
   ___XXX__,_XXXX___,
   ___XXX__,_XXX____,
   ___XXX__,_XXX____,
   ___XXX__,_XXX____,
   ___XXX__,_XXX____,
   ________,________,
   ________,________,
   ________,________}
// 0x58  X
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   _XXX___X,XX______,
   __XXX_XX,X_______,
   ___XXXXX,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ____XXX_,________,
   ___XXXXX,________,
   __XXX_XX,X_______,
   _XXX___X,XX______,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   XXX_____,XXX_____,
   ________,________,
   ________,________,
   ________,________}
// 0x59  Y
 ,{
   14,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   XXX_____,_XXX____,
   _XXX____,XXX_____,
   _XXX____,XXX_____,
   __XXX__X,XX______,
   __XXX__X,XX______,
   ___XXXXX,X_______,
   ____XXXX,________,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   _____XXX,________,
   ________,________,
   ________,________,
   ________,________}
// 0x5A  Z
 ,{
   12,
   f24_FLOAT_HEIGHT,
   ________,________,
   ________,________,
   ________,________,
   XXXXXXXX,X_______,
   XXXXXXXX,X_______,
   XXXXXXXX,X_______,
   ______XX,X_______,
   ______XX,X_______,
   _____XXX,________,
   _____XXX,________,
   _____XX_,________,
   ____XXX_,________,
   ____XX__,________,
   ___XXX__,________,
   __XXX___,________,
   __XXX___,________,
   _XXX____,________,
   XXX_____,________,
   XXXXXXXX,X_______,
   XXXXXXXX,X_______,
   XXXXXXXX,X_______,
   ________,________,
   ________,________,
   ________,________}
};
const uint8_t f24f_table1[48 + 2] =
// 0xB0
{
  10,
  f24_FLOAT_HEIGHT,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  __XXXXX_,________,
  _XXXXXXX,________,
  XXX___XX,X_______,
  XXX___XX,X_______,
  XXX___XX,X_______,
  _XXXXXXX,________,
  __XXXXX_,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________
/*
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ____XXXX,________,
  __XXXXXX,XX______,
  __XXX__X,XX______,
  _XXX____,XXX_____,
  _XXX____,XXX_____,
  _XXX____,XXX_____,
  __XXX__X,XX______,
  __XXXXXX,XX______,
  ____XXXX,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________,
  ________,________
*/
};

//==============================================================================
// ������� ���������� ��������� �� ���������� ������� Char
// ������ �����
//==============================================================================
uint8_t *f24f_GetCharTable(char Char)
{
	// ������ �����
	if ((Char >= 0x20) && (Char <= 0x5A))
	    return (uint8_t *)(&f24f_table[Char - 0x20][0]);
	if (Char == 0xB0)
		return (uint8_t *) f24f_table1;

	return 0;
}
//==============================================================================
