#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <memory>

#include "esp_log.h"

#include "rbjson.h"
#include "jsmn.h"

#define TAG "RbJson"

namespace rbjson {

static int count_tok_size(jsmntok_t *tok) {
    jsmntok_t *itr = tok+1;
    for(int i = 0; i < tok->size; ++i) {
        itr += count_tok_size(itr);
    }
    return itr-tok;
}

static void write_string_escaped(const char *str, std::stringstream& ss) {
    const char *start = str;
    const char *end = NULL;
    ss << '"';
    while(true) {
        end = strchr(start, '"');
        if(end == NULL) {
            ss << start;
            ss << '"';
            return;
        } else {
            ss.write(start, end-start);
            ss << "\\\"";
            start = end+1;
        }
    }
}

static Value *parse_value(char *buf, jsmntok_t *tok);

static Object *parse_object(char *buf, jsmntok_t *obj) {
    if(obj->type != JSMN_OBJECT) {
        return NULL;
    }

    Object *res = new Object();
    jsmntok_t *tok = obj + 1;
    for(int i = 0; i < obj->size; ++i) {
        if(tok->type != JSMN_STRING || tok->size != 1) {
            continue;
        }
        
        Value *val = parse_value(buf, tok+1);
        if(val != NULL) {
            std::string key(buf + tok->start, tok->end - tok->start);
            res->set(key.c_str(), val);
        }

        tok += count_tok_size(tok);
    }
    return res;
}

static Array *parse_array(char *buf, jsmntok_t *arr) {
    if(arr->type != JSMN_ARRAY) {
        return NULL;
    }

    Array *res = new Array();
    jsmntok_t *tok = arr + 1;
    for(int i = 0; i < arr->size; ++i) {      
        Value *val = parse_value(buf, tok);
        if(val != NULL) {
            res->push_back(val);
        }
        tok += count_tok_size(tok);
    }
    return res;
}

Value *parse_value(char *buf, jsmntok_t *tok) {
    switch(tok->type) {
    case JSMN_OBJECT:
        return parse_object(buf, tok);
    case JSMN_ARRAY:
        return parse_array(buf, tok);
    case JSMN_STRING:
        return new String(std::string(buf + tok->start, tok->end - tok->start));
    case JSMN_PRIMITIVE: {
        const char *str = buf + tok->start;
        const int len = tok->end - tok->start;
        if(len == 0) {
            return NULL;
        }

        switch(*str) {
        case 't':
            return new Bool(true);
        case 'f':
            return new Bool(false);
        case 'n':
            return new Nil();
        default: {
            char buf[32];
            snprintf(buf, sizeof(buf), "%.*s", len, str);

            char *endptr;
            double val = strtod(buf, &endptr);
            if(buf == endptr) {
                return NULL;
            }
            return new Number(val);
        }
        }
    }
    default:
        return NULL;
    }
}

Object *parse(char *buf, size_t size) {
    jsmn_parser parser;
    size_t tokens_size = 32;
    jsmntok_t tokens_static[32];
    std::unique_ptr<jsmntok_t> tokens_dynamic;
    jsmntok_t *tokens = tokens_static;
    int parsed;

    while (true) {
        jsmn_init(&parser);
        parsed = jsmn_parse(&parser, buf, size, tokens, tokens_size);
        if(parsed >= 0) {
            break;
        } else if(parsed == JSMN_ERROR_NOMEM) {
            tokens_size *= 2;
            if(tokens_size >= 128) {
                ESP_LOGE(TAG, "failed to parse msg %.*s: too big", size, buf);
                return NULL;
            }
            tokens_dynamic.reset(new jsmntok_t[tokens_size]);
            tokens = tokens_dynamic.get();
        } else {
            ESP_LOGE(TAG, "failed to parse msg %.*s: %d", size, buf, parsed);
            return NULL;
        }
    }
    return parse_object(buf, &tokens[0]);
}

Value::Value(Value::type_t type) : m_type(type) {

}

Value::~Value() {

}

std::string Value::str() const {
    std::stringstream ss;
    this->serialize(ss);
    return ss.str();
}

Object::Object() : Value(Value::OBJECT) {

}

Object::~Object() {
    for(auto itr = m_members.begin(); itr != m_members.end(); ++itr) {
        delete itr->second;
    }
}

void Object::serialize(std::stringstream& ss) const {
    ss << '{';
    for(auto itr = m_members.begin(); itr != m_members.end();) {
        write_string_escaped(itr->first.c_str(), ss);
        ss << ':';
        itr->second->serialize(ss);
        if(++itr != m_members.end()) {
            ss << ',';
        }
    }
    ss << '}';
}

bool Object::contains(const char *key) const {
    return m_members.find(key) != m_members.end();
}

Value *Object::get(const char *key) const {
    const auto itr = m_members.find(key);
    if(itr == m_members.cend())
        return NULL;
    return itr->second;
}

Object *Object::getObject(const char *key) const {
    auto *val = get(key);
    if(val && val->getType() == OBJECT) {
        return (Object*)val;
    }
    return NULL;
}

Array *Object::getArray(const char *key) const {
    auto *val = get(key);
    if(val && val->getType() == ARRAY) {
        return (Array*)val;
    }
    return NULL;
}

std::string Object::getString(const char *key, std::string def) const {
    auto *val = get(key);
    if(val && val->getType() == STRING) {
        return ((String*)val)->get();
    } else {
        return def;
    }
}

int64_t Object::getInt(const char *key, int64_t def) const {
    auto *val = get(key);
    if(val && val->getType() == NUMBER) {
        return ((Number*)val)->get();
    } else {
        return def;
    }
}

double Object::getDouble(const char *key, double def) const {
    auto *val = get(key);
    if(val && val->getType() == NUMBER) {
        return ((Number*)val)->get();
    } else {
        return def;
    }
}

bool Object::getBool(const char *key, bool def) const {
    auto *val = get(key);
    if(val && val->getType() == BOOL) {
        return ((Bool*)val)->get();
    } else {
        return def;
    }
}

void Object::set(const char *key, Value *value) {
    auto itr = m_members.find(key);
    if(itr != m_members.end()) {
        delete itr->second;
        itr->second = value;
    } else {
        m_members[key] = value;
    }
}

void Object::set(const char *key, const char *string) {
    set(key, new String(string));
}

void Object::set(const char *key, const std::string& str) {
    set(key, new String(str));
}

void Object::set(const char *key, int64_t number) {
    set(key, new Number(number));
}

void Object::remove(const char *key) {
    auto itr = m_members.find(key);
    if(itr != m_members.end()) {
        delete itr->second;
        m_members.erase(itr);
    } 
}

Array::Array() : Value(Value::ARRAY) {

}

Array::~Array() {
    for(auto val : m_items) {
        delete val;
    }
}

void Array::serialize(std::stringstream& ss) const {
    ss << '[';
    for(size_t i = 0; i < m_items.size(); ++i) {
        m_items[i]->serialize(ss);
        if(i+1 != m_items.size()) {
            ss << ',';
        }
    }
    ss << ']';
}

Value *Array::get(size_t idx) const {
    if(idx < m_items.size())
        return m_items[idx];
    return NULL;
}

Object *Array::getObject(size_t idx) const {
    auto *val = get(idx);
    if(val && val->getType() == OBJECT) {
        return (Object*)val;
    }
    return NULL;
}

Array *Array::getArray(size_t idx) const {
    auto *val = get(idx);
    if(val && val->getType() == ARRAY) {
        return (Array*)val;
    }
    return NULL;
}

std::string Array::getString(size_t idx, std::string def) const {
    auto *val = get(idx);
    if(val && val->getType() == STRING) {
        return ((String*)val)->get();
    } else {
        return def;
    }
}

int64_t Array::getInt(size_t idx, int64_t def) const {
    auto *val = get(idx);
    if(val && val->getType() == NUMBER) {
        return ((Number*)val)->get();
    } else {
        return def;
    }
}

double Array::getDouble(size_t idx, double def) const {
    auto *val = get(idx);
    if(val && val->getType() == NUMBER) {
        return ((Number*)val)->get();
    } else {
        return def;
    }
}

bool Array::getBool(size_t idx, bool def) const {
    auto *val = get(idx);
    if(val && val->getType() == BOOL) {
        return ((Bool*)val)->get();
    } else {
        return def;
    }
}

void Array::set(size_t idx, Value *value) {
    if(idx < m_items.size()) {
        delete m_items[idx];
        m_items[idx] = value;
    }
}

void Array::insert(size_t idx, Value *value) {
    m_items.insert(m_items.begin()+idx, value);
}

void Array::remove(size_t idx) {
    if(idx < m_items.size()) {
        delete m_items[idx];
        m_items.erase(m_items.begin() + idx);
    }
}

String::String(const char *value) : Value(STRING), m_value(value) {

}

String::String(const std::string& value) : Value(STRING), m_value(value) {

}

String::~String() {

}

void String::serialize(std::stringstream& ss) const {
    write_string_escaped(m_value.c_str(), ss);
}

Number::Number(double value) : Value(NUMBER), m_value(value) {

}

Number::~Number() {

}

void Number::serialize(std::stringstream& ss) const {
    double intpart;
    double fracpart = modf(m_value, &intpart);
    if(fracpart > -0.0001 && fracpart < 0.0001) {
        ss << int64_t(m_value);
    } else {
        ss << m_value;
    }
}

Bool::Bool(bool value) : Value(BOOL), m_value(value) {

}

Bool::~Bool() {

}

void Bool::serialize(std::stringstream& ss) const {
    if(m_value) {
        ss << "true";
    } else {
        ss << "false";
    }
}

void Nil::serialize(std::stringstream& ss) const {
    ss << "null";
}


};