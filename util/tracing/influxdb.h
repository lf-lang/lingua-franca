#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/uio.h>
#include <netdb.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <curl/curl.h>

/*
  Usage:
    send_udp/post_http(c,
            INFLUX_MEAS("foo"),
            INFLUX_TAG("k", "v"), INFLUX_TAG("k2", "v2"),
            INFLUX_F_STR("s", "string"), INFLUX_F_FLT("f", 28.39, 2),

            INFLUX_MEAS("bar"),
            INFLUX_F_INT("i", 1048576), INFLUX_F_BOL("b", 1),
            INFLUX_TS(1512722735522840439),

            INFLUX_END);

  **NOTICE**: For best performance you should sort tags by key before sending them to the database.
              The sort should match the results from the [Go bytes.Compare function](https://golang.org/pkg/bytes/#Compare).
 */

#define INFLUX_MEAS(m)        IF_TYPE_MEAS, (m)
#define INFLUX_TAG(k, v)      IF_TYPE_TAG, (k), (v)
#define INFLUX_F_STR(k, v)    IF_TYPE_FIELD_STRING, (k), (v)
#define INFLUX_F_FLT(k, v, p) IF_TYPE_FIELD_FLOAT, (k), (double)(v), (int)(p)
#define INFLUX_F_INT(k, v)    IF_TYPE_FIELD_INTEGER, (k), (long long)(v)
#define INFLUX_F_BOL(k, v)    IF_TYPE_FIELD_BOOLEAN, (k), ((v) ? 1 : 0)
#define INFLUX_TS(ts)         IF_TYPE_TIMESTAMP, (long long)(ts)
#define INFLUX_END            IF_TYPE_ARG_END

typedef struct _influx_client_t
{
    char* host;
    int   port;
    char* db;  // http only
    char* usr; // http only [optional for auth]
    char* pwd; // http only [optional for auth]
    char* token; // http only
} influx_client_t;

typedef struct _influx_v2_client_t
{
    char* host;
    int   port;
    char* org;  
    char* bucket;
    char* precision;
    char* usr; // http only [optional for auth]
    char* pwd; // http only [optional for auth]
    char* token; // http only
} influx_v2_client_t;

int format_line(char **buf, int *len, size_t used, ...);
int post_http(influx_client_t* c, ...);
int send_udp(influx_client_t* c, ...);
int post_curl(influx_v2_client_t* c, ...);

#define IF_TYPE_ARG_END       0
#define IF_TYPE_MEAS          1
#define IF_TYPE_TAG           2
#define IF_TYPE_FIELD_STRING  3
#define IF_TYPE_FIELD_FLOAT   4
#define IF_TYPE_FIELD_INTEGER 5
#define IF_TYPE_FIELD_BOOLEAN 6
#define IF_TYPE_TIMESTAMP     7

int _escaped_append(char** dest, size_t* len, size_t* used, const char* src, const char* escape_seq);
int _begin_line(char **buf);
int _format_line(char** buf, va_list ap);
int _format_line2(char** buf, va_list ap, size_t *, size_t);
int post_http_send_line(influx_client_t *c, char *buf, int len);
int send_udp_line(influx_client_t* c, char *line, int len);

int post_http_send_line(influx_client_t *c, char *buf, int len)
{
    int sock = -1   , ret_code = 0, content_length = 0;
    struct sockaddr_in addr;
    struct iovec iv[2];
    char ch;

    iv[1].iov_base = buf;
    iv[1].iov_len = len;

    if(!(iv[0].iov_base = (char*)malloc(len = 0x800))) {
        free(iv[1].iov_base);
        return -2;
    }

    for(;;) {
        iv[0].iov_len = snprintf((char*)iv[0].iov_base, len, 
            "POST /write?db=%s&u=%s&p=%s HTTP/1.1\r\n"
            "Host: %s\r\n"
            "Accept: application/json\r\n"
            "Content-type: text/plain\r\n"
            "Authorization: Token %s\r\n"
            "Content-Length: %zd\r\n"
            "\r\n", // Final blank line is needed.
            c->db, c->usr ? c->usr : "", c->pwd ? c->pwd : "", c->host, c->token ? c->token : "", iv[1].iov_len);
        if((int)iv[0].iov_len >= len && !(iv[0].iov_base = (char*)realloc(iv[0].iov_base, len *= 2))) {
            free(iv[1].iov_base);
            free(iv[0].iov_base);
            return -3;
        }
        else
            break;
    }

	fprintf(stderr, "influxdb-c::post_http: iv[0] = '%s'\n", (char *)iv[0].iov_base);
	fprintf(stderr, "influxdb-c::post_http: iv[1] = '%s'\n", (char *)iv[1].iov_base);

    addr.sin_family = AF_INET;
    addr.sin_port = htons(c->port);
    // EAL: Rather than just an IP address, allow a hostname, like "localhost"
    struct hostent* resolved_host = gethostbyname(c->host);
    if (!resolved_host) {
        free(iv[1].iov_base);
        free(iv[0].iov_base);
        return -4;
    }
    memcpy(&addr.sin_addr, resolved_host->h_addr_list[0], resolved_host->h_length);
    /*
    if((addr.sin_addr.s_addr = inet_addr(resolved_host->h_addr)) == INADDR_NONE) {
        free(iv[1].iov_base);
        free(iv[0].iov_base);
        return -4;
    }
    */

    if((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        free(iv[1].iov_base);
        free(iv[0].iov_base);
        return -5;
    }

    if(connect(sock, (struct sockaddr*)(&addr), sizeof(addr)) < 0) {
        ret_code = -6;
        goto END;
    }

    if(writev(sock, iv, 2) < (int)(iv[0].iov_len + iv[1].iov_len)) {
        ret_code = -7;
        goto END;
    }
    iv[0].iov_len = len;

#define _GET_NEXT_CHAR() (ch = (len >= (int)iv[0].iov_len && \
    (iv[0].iov_len = recv(sock, iv[0].iov_base, iv[0].iov_len, len = 0)) == (size_t)(-1) ? \
     0 : *((char*)iv[0].iov_base + len++)))
#define _LOOP_NEXT(statement) for(;;) { if(!(_GET_NEXT_CHAR())) { ret_code = -8; goto END; } statement }
#define _UNTIL(c) _LOOP_NEXT( if(ch == c) break; )
#define _GET_NUMBER(n) _LOOP_NEXT( if(ch >= '0' && ch <= '9') n = n * 10 + (ch - '0'); else break; )
#define _(c) if((_GET_NEXT_CHAR()) != c) break;

    _UNTIL(' ')_GET_NUMBER(ret_code)
    for(;;) {
        _UNTIL('\n')
        switch(_GET_NEXT_CHAR()) {
            case 'C':_('o')_('n')_('t')_('e')_('n')_('t')_('-')
                _('L')_('e')_('n')_('g')_('t')_('h')_(':')_(' ')
                _GET_NUMBER(content_length)
                break;
            case '\r':_('\n')
                while(content_length-- > 0 && _GET_NEXT_CHAR());// printf("%c", ch);
                goto END;
        }
        if(!ch) {
            ret_code = -10;
            goto END;
        }
    }
    ret_code = -11;
END:
    close(sock);
    free(iv[0].iov_base);
    free(iv[1].iov_base);
    return ret_code / 100 == 2 ? 0 : ret_code;
}
#undef _GET_NEXT_CHAR
#undef _LOOP_NEXT
#undef _UNTIL
#undef _GET_NUMBER
#undef _

int post_http(influx_client_t* c, ...)
{
    va_list ap;
    char *line = NULL;
    int ret_code = 0, len = 0;

    va_start(ap, c);
    len = _format_line((char**)&line, ap);
    va_end(ap);
    if(len < 0)
        return -1;

    ret_code = post_http_send_line(c, line, len);

    return ret_code;
}

int send_udp_line(influx_client_t* c, char *line, int len)
{
    int sock = -1, ret = 0;
    struct sockaddr_in addr;

    addr.sin_family = AF_INET;
    addr.sin_port = htons(c->port);
    if((addr.sin_addr.s_addr = inet_addr(c->host)) == INADDR_NONE) {
        ret = -2;
        goto END;
    }

    if((sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        ret = -3;
        goto END;
    }

    if(sendto(sock, line, len, 0, (struct sockaddr *)&addr, sizeof(addr)) < len)
        ret = -4;

END:
    if (sock >= 0) {
        close(sock);
    }
    return ret;
}

int send_udp(influx_client_t* c, ...)
{
    int ret = 0, len;
    va_list ap;
    char* line = NULL;

    va_start(ap, c);
    len = _format_line(&line, ap);
    va_end(ap);
    if(len < 0)
        return -1;

    ret = send_udp_line(c, line, len);

    free(line);
    return ret;
}

int post_curl(influx_v2_client_t* c, ...)
{
    va_list ap;
    char *data = NULL;
    int len = 0;
    va_start(ap, c);
    len = _format_line((char**)&data, ap);
    va_end(ap);

    CURL *curl;

    /* In windows, this will init the winsock stuff */ 
    curl_global_init(CURL_GLOBAL_ALL);
    CURLcode res;

    /* get a curl handle */ 
    curl = curl_easy_init();
    if(!curl) {
        return CURLE_FAILED_INIT;
    }

    char* url_string = (char*)malloc(len);
    snprintf(url_string, len, 
            "http://%s:%d/api/v2/write?org=%s&bucket=%s&precision=%s",
            c->host ? c->host: "localhost", c->port ? c->port : 8086, c->org, c->bucket, c->precision ? c->precision : "ns");

    curl_easy_setopt(curl, CURLOPT_URL, url_string);   
    free(url_string);
            
    char* token_string = (char*)malloc(120*sizeof(char));
    sprintf(token_string, "Authorization: Token %s", c->token);

    struct curl_slist *list = NULL;
    list = curl_slist_append(list, token_string);
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, list);
    free(token_string);
    
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");

    /* Perform the request, res will get the return code */ 
    res = curl_easy_perform(curl);
    /* Check for errors */ 
    if(res != CURLE_OK){
        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                curl_easy_strerror(res));   
    }
        
    free(data);
    curl_easy_cleanup(curl);
    curl_global_cleanup();
    return res;
}

int format_line(char **buf, int *len, size_t used, ...)
{
    va_list ap;
    va_start(ap, used);
    used = _format_line2(buf, ap, (size_t *)len, used);
    va_end(ap);
    if(*len < 0)
        return -1;
    else
	return used;
}

int _begin_line(char **buf)
{
    int len = 0x100;
    if(!(*buf = (char*)malloc(len)))
        return -1;
    return len;
}

int _format_line(char** buf, va_list ap)
{
	size_t len = 0;
	*buf = NULL;
	return _format_line2(buf, ap, &len, 0);
}

int _format_line2(char** buf, va_list ap, size_t *_len, size_t used)
{
#define _APPEND(fmter...) \
    for(;;) {\
        if((written = snprintf(*buf + used, len - used, ##fmter)) < 0)\
            goto FAIL;\
        if(used + written >= len && !(*buf = (char*)realloc(*buf, len *= 2)))\
            return -1;\
        else {\
            used += written;\
            break;\
        }\
    }

    size_t len = *_len;
    int written = 0, type = 0, last_type = 0;
    unsigned long long i = 0;
    double d = 0.0;

    if (*buf == NULL) {
	    len = _begin_line(buf);
	    used = 0;
    }

    type = va_arg(ap, int);
    while(type != IF_TYPE_ARG_END) {
        if(type >= IF_TYPE_TAG && type <= IF_TYPE_FIELD_BOOLEAN) {
            if(last_type < IF_TYPE_MEAS || last_type > (type == IF_TYPE_TAG ? IF_TYPE_TAG : IF_TYPE_FIELD_BOOLEAN))
                goto FAIL;
            _APPEND("%c", (last_type <= IF_TYPE_TAG && type > IF_TYPE_TAG) ? ' ' : ',');
            if(_escaped_append(buf, &len, &used, va_arg(ap, char*), ",= "))
                return -2;
            _APPEND("=");
        }
        switch(type) {
            case IF_TYPE_MEAS:
                if(last_type)
                    _APPEND("\n");
                if(last_type && last_type <= IF_TYPE_TAG)
                    goto FAIL;
                if(_escaped_append(buf, &len, &used, va_arg(ap, char*), ", "))
                    return -3;
                break;
            case IF_TYPE_TAG:
                if(_escaped_append(buf, &len, &used, va_arg(ap, char*), ",= "))
                    return -4;
                break;
            case IF_TYPE_FIELD_STRING:
                _APPEND("\"");
                if(_escaped_append(buf, &len, &used, va_arg(ap, char*), "\""))
                    return -5;
                _APPEND("\"");
                break;
            case IF_TYPE_FIELD_FLOAT:
                d = va_arg(ap, double);
                i = va_arg(ap, int);
                _APPEND("%.*lf", (int)i, d);
                break;
            case IF_TYPE_FIELD_INTEGER:
                i = va_arg(ap, long long);
                _APPEND("%lldi", i);
                break;
            case IF_TYPE_FIELD_BOOLEAN:
                i = va_arg(ap, int);
                _APPEND("%c", i ? 't' : 'f');
                break;
            case IF_TYPE_TIMESTAMP:
                if(last_type < IF_TYPE_FIELD_STRING || last_type > IF_TYPE_FIELD_BOOLEAN)
                    goto FAIL;
                i = va_arg(ap, long long);
                _APPEND(" %lld", i);
                break;
            default:
                goto FAIL;
        }
        last_type = type;
        type = va_arg(ap, int);
    }
    _APPEND("\n");
    if(last_type <= IF_TYPE_TAG)
        goto FAIL;
    *_len = len;
    return used;
FAIL:
    free(*buf);
    *buf = NULL;
    return -1;
}
#undef _APPEND

int _escaped_append(char** dest, size_t* len, size_t* used, const char* src, const char* escape_seq)
{
    size_t i = 0;

    for(;;) {
        if((i = strcspn(src, escape_seq)) > 0) {
            if(*used + i > *len && !(*dest = (char*)realloc(*dest, (*len) *= 2)))
                return -1;
            strncpy(*dest + *used, src, i);
            *used += i;
            src += i;
        }
        if(*src) {
            if(*used + 2 > *len && !(*dest = (char*)realloc(*dest, (*len) *= 2)))
                return -2;
            (*dest)[(*used)++] = '\\';
            (*dest)[(*used)++] = *src++;
        }
        else
            return 0;
    }
    return 0;
}
