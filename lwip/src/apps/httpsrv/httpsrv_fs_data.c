
#include "global_config.h"
#include <httpsrv_fs.h>


#ifndef  WEB_FUNCTION_ENABLE
#define  WEB_FUNCTION_ENABLE		1
#endif


#include "webpage/index.html.h"

#if WEB_FUNCTION_ENABLE

#include "webpage/0.js.h"
#include "webpage/1.js.h"
#include "webpage/2.js.h"
#include "webpage/3.js.h"
#include "webpage/4.js.h"
#include "webpage/5.js.h"
#include "webpage/6.js.h"
#include "webpage/manifest.js.h"
#include "webpage/vendor.js.h"
#include "webpage/app.js.h"
#include "webpage/babel-polyfill.js.h"
#include "webpage/common.js.h"
#include "webpage/element-icons.ttf.h"
#include "webpage/favicon.ico.h"
#include "webpage/app.css.h"
#include "webpage/head_img.png.h"

#endif

extern const HTTPSRV_FS_DIR_ENTRY httpsrv_fs_data[];

const HTTPSRV_FS_DIR_ENTRY httpsrv_fs_data[] = {
	{ "/index.html", 0, (unsigned char*)httpsrv_fs_index_html, sizeof(httpsrv_fs_index_html) },
#if WEB_FUNCTION_ENABLE
	{ "/static/common.js", 0, (unsigned char*)httpsrv_fs_common_js, sizeof(httpsrv_fs_common_js) },
	{ "/static/favicon.ico", 0, (unsigned char*)httpsrv_fs_favicon_ico, sizeof(httpsrv_fs_favicon_ico) },
	{ "/static/css/app.css", 0, (unsigned char*)httpsrv_fs_app_css, sizeof(httpsrv_fs_app_css) },
	{ "/static/fonts/element-icons.ttf", 0, (unsigned char*)httpsrv_fs_element_icons_ttf, sizeof(httpsrv_fs_element_icons_ttf) },
	{ "/static/img/head_img.png", 0, (unsigned char*)httpsrv_fs_head_img_png, sizeof(httpsrv_fs_head_img_png) },
	{ "/static/js/0.js", 0, (unsigned char*)httpsrv_fs_0_js, sizeof(httpsrv_fs_0_js) },
	{ "/static/js/1.js", 0, (unsigned char*)httpsrv_fs_1_js, sizeof(httpsrv_fs_1_js) },
	{ "/static/js/2.js", 0, (unsigned char*)httpsrv_fs_2_js, sizeof(httpsrv_fs_2_js) },
	{ "/static/js/3.js", 0, (unsigned char*)httpsrv_fs_3_js, sizeof(httpsrv_fs_3_js) },
	{ "/static/js/4.js", 0, (unsigned char*)httpsrv_fs_4_js, sizeof(httpsrv_fs_4_js) },
	{ "/static/js/5.js", 0, (unsigned char*)httpsrv_fs_5_js, sizeof(httpsrv_fs_5_js) },
	{ "/static/js/6.js", 0, (unsigned char*)httpsrv_fs_6_js, sizeof(httpsrv_fs_6_js) },
	{ "/static/js/app.js", 0, (unsigned char*)httpsrv_fs_app_js, sizeof(httpsrv_fs_app_js) },
	{ "/static/js/babel-polyfill.js", 0, (unsigned char*)httpsrv_fs_babel_polyfill_js, sizeof(httpsrv_fs_babel_polyfill_js) },
	{ "/static/js/manifest.js", 0, (unsigned char*)httpsrv_fs_manifest_js, sizeof(httpsrv_fs_manifest_js) },
	{ "/static/js/vendor.js", 0, (unsigned char*)httpsrv_fs_vendor_js, sizeof(httpsrv_fs_vendor_js) },
#endif	
	{ 0, 0, 0, 0 }
};

