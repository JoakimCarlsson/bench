//! The engine's JSON value tree and strict recursive descent parser, ported by
//! hand. Objects keep members in insertion order.

/// A JSON value: null, boolean, number, string, array or object.
pub enum Value {
    Null,
    Bool(bool),
    Number(f64),
    String(String),
    Array(Vec<Value>),
    Object(Vec<(String, Value)>),
}

/// A parse failure: malformed input, trailing content or nesting too deep.
pub struct ParseError;

/// Deepest nesting the parser accepts.
const MAX_DEPTH: usize = 128;

impl Value {
    /// The array elements; empty for any other kind.
    pub fn items(&self) -> &[Value] {
        match self {
            Value::Array(items) => items,
            _ => &[],
        }
    }

    /// The object members in insertion order; empty for any other kind.
    pub fn members(&self) -> &[(String, Value)] {
        match self {
            Value::Object(members) => members,
            _ => &[],
        }
    }

    /// The string, or `fallback` for any other kind.
    pub fn as_str_or<'a>(&'a self, fallback: &'a str) -> &'a str {
        match self {
            Value::String(text) => text,
            _ => fallback,
        }
    }

    /// Sets an object member, replacing an existing one. Does nothing for any
    /// other kind.
    fn set(&mut self, key: String, value: Value) {
        let Value::Object(members) = self else { return };
        for member in members.iter_mut() {
            if member.0 == key {
                member.1 = value;
                return;
            }
        }
        members.push((key, value));
    }

    /// Looks up an object member; `None` when absent.
    pub fn find(&self, key: &str) -> Option<&Value> {
        self.members().iter().find(|member| member.0 == key).map(|member| &member.1)
    }

    /// Reads a boolean member, or `fallback` when absent or not a boolean.
    pub fn bool_or(&self, key: &str, fallback: bool) -> bool {
        match self.find(key) {
            Some(Value::Bool(value)) => *value,
            _ => fallback,
        }
    }

    /// Reads a number member, or `fallback` when absent or not a number.
    pub fn number_or(&self, key: &str, fallback: f64) -> f64 {
        match self.find(key) {
            Some(Value::Number(value)) => *value,
            _ => fallback,
        }
    }

    /// Reads a string member without copying it, or `fallback` when absent or
    /// not a string.
    pub fn text_or<'a>(&'a self, key: &str, fallback: &'a str) -> &'a str {
        match self.find(key) {
            Some(Value::String(text)) => text,
            _ => fallback,
        }
    }
}

/// Parses JSON text. Fails on malformed input, trailing content or nesting
/// deeper than 128.
pub fn parse(source: &str) -> Result<Value, ParseError> {
    let mut parser = Parser { source: source.as_bytes(), offset: 0 };
    parser.skip_space();
    let value = parser.parse_value(0)?;
    parser.skip_space();
    if parser.offset != parser.source.len() {
        return Err(parser.fail());
    }
    Ok(value)
}

/// A recursive descent parser over bytes.
struct Parser<'a> {
    source: &'a [u8],
    offset: usize,
}

impl Parser<'_> {
    /// The error for malformed input.
    fn fail(&self) -> ParseError {
        ParseError
    }

    /// Advances past JSON whitespace.
    fn skip_space(&mut self) {
        while self.offset < self.source.len() {
            match self.source[self.offset] {
                b' ' | b'\t' | b'\n' | b'\r' => self.offset += 1,
                _ => break,
            }
        }
    }

    /// Consumes `character` if it is next.
    fn expect(&mut self, character: u8) -> Result<(), ParseError> {
        if self.offset >= self.source.len() || self.source[self.offset] != character {
            return Err(self.fail());
        }
        self.offset += 1;
        Ok(())
    }

    /// Tests the next character without consuming it.
    fn next_is(&self, character: u8) -> bool {
        self.offset < self.source.len() && self.source[self.offset] == character
    }

    /// Consumes a keyword when the input continues with it.
    fn literal(&mut self, word: &[u8]) -> Result<(), ParseError> {
        if !self.source[self.offset..].starts_with(word) {
            return Err(self.fail());
        }
        self.offset += word.len();
        Ok(())
    }

    /// Parses any value at the current position.
    fn parse_value(&mut self, depth: usize) -> Result<Value, ParseError> {
        if depth > MAX_DEPTH || self.offset >= self.source.len() {
            return Err(self.fail());
        }
        match self.source[self.offset] {
            b'{' => self.parse_object(depth),
            b'[' => self.parse_array(depth),
            b'"' => Ok(Value::String(self.parse_string()?)),
            b't' => {
                self.literal(b"true")?;
                Ok(Value::Bool(true))
            }
            b'f' => {
                self.literal(b"false")?;
                Ok(Value::Bool(false))
            }
            b'n' => {
                self.literal(b"null")?;
                Ok(Value::Null)
            }
            _ => Ok(Value::Number(self.parse_number()?)),
        }
    }

    /// Parses an object.
    fn parse_object(&mut self, depth: usize) -> Result<Value, ParseError> {
        self.expect(b'{')?;
        let mut result = Value::Object(Vec::new());
        self.skip_space();
        if self.next_is(b'}') {
            self.offset += 1;
            return Ok(result);
        }
        loop {
            self.skip_space();
            let key = self.parse_string()?;
            self.skip_space();
            self.expect(b':')?;
            self.skip_space();
            let member = self.parse_value(depth + 1)?;
            result.set(key, member);
            self.skip_space();
            if self.next_is(b',') {
                self.offset += 1;
                continue;
            }
            self.expect(b'}')?;
            return Ok(result);
        }
    }

    /// Parses an array.
    fn parse_array(&mut self, depth: usize) -> Result<Value, ParseError> {
        self.expect(b'[')?;
        let mut items = Vec::new();
        self.skip_space();
        if self.next_is(b']') {
            self.offset += 1;
            return Ok(Value::Array(items));
        }
        loop {
            self.skip_space();
            items.push(self.parse_value(depth + 1)?);
            self.skip_space();
            if self.next_is(b',') {
                self.offset += 1;
                continue;
            }
            self.expect(b']')?;
            return Ok(Value::Array(items));
        }
    }

    /// Parses a quoted string and decodes its escapes. Fails when the decoded
    /// bytes are not UTF-8, which a `String` cannot hold.
    fn parse_string(&mut self) -> Result<String, ParseError> {
        self.expect(b'"')?;
        let mut result: Vec<u8> = Vec::new();
        loop {
            if self.offset >= self.source.len() {
                return Err(self.fail());
            }
            let character = self.source[self.offset];
            self.offset += 1;
            if character == b'"' {
                return String::from_utf8(result).map_err(|_| self.fail());
            }
            if character != b'\\' {
                result.push(character);
                continue;
            }
            if self.offset >= self.source.len() {
                return Err(self.fail());
            }
            let escape = self.source[self.offset];
            self.offset += 1;
            match escape {
                b'"' | b'\\' | b'/' => result.push(escape),
                b'b' => result.push(0x08),
                b'f' => result.push(0x0C),
                b'n' => result.push(b'\n'),
                b'r' => result.push(b'\r'),
                b't' => result.push(b'\t'),
                b'u' => self.parse_escape_unit(&mut result)?,
                _ => return Err(self.fail()),
            }
        }
    }

    /// Decodes the four hex digits of a unicode escape to UTF-8 and appends
    /// them. Surrogate pairs are not combined.
    fn parse_escape_unit(&mut self, out: &mut Vec<u8>) -> Result<(), ParseError> {
        if self.offset + 4 > self.source.len() {
            return Err(self.fail());
        }
        let mut code: u32 = 0;
        for _ in 0..4 {
            let digit = self.source[self.offset];
            self.offset += 1;
            code <<= 4;
            code |= match digit {
                b'0'..=b'9' => u32::from(digit - b'0'),
                b'a'..=b'f' => u32::from(digit - b'a') + 10,
                b'A'..=b'F' => u32::from(digit - b'A') + 10,
                _ => return Err(self.fail()),
            };
        }
        if code < 0x80 {
            out.push(code as u8);
        } else if code < 0x800 {
            out.push((0xC0 | (code >> 6)) as u8);
            out.push((0x80 | (code & 0x3F)) as u8);
        } else {
            out.push((0xE0 | (code >> 12)) as u8);
            out.push((0x80 | ((code >> 6) & 0x3F)) as u8);
            out.push((0x80 | (code & 0x3F)) as u8);
        }
        Ok(())
    }

    /// Parses a number token with `str::parse`, which rounds correctly.
    fn parse_number(&mut self) -> Result<f64, ParseError> {
        let start = self.offset;
        if self.offset < self.source.len() && self.source[self.offset] == b'-' {
            self.offset += 1;
        }
        while self.offset < self.source.len() {
            match self.source[self.offset] {
                b'0'..=b'9' | b'.' | b'e' | b'E' | b'+' | b'-' => self.offset += 1,
                _ => break,
            }
        }
        if self.offset == start {
            return Err(self.fail());
        }
        std::str::from_utf8(&self.source[start..self.offset])
            .ok()
            .and_then(|token| token.parse::<f64>().ok())
            .ok_or_else(|| self.fail())
    }
}
