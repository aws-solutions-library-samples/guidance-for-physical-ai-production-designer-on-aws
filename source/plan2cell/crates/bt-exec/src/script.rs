//! The subset of BehaviorTree.CPP v4's scripting language used by `Script`, `ScriptCondition`,
//! `Precondition` and the `_skipIf`-style conditions.
//!
//! Statements are separated by `;`. Supported: `:=` / `=` / `+=` / `-=` / `*=` / `/=`,
//! `|| && !`, `== != < <= > >=`, `+ - * /` (with `+` concatenating strings), `? :`,
//! parentheses, numbers, `'strings'` / `"strings"`, `true` / `false` and blackboard names.

use serde_json::Value;

/// Where scripts read and write variables.
pub trait Env {
    fn get(&self, key: &str) -> Option<Value>;
    fn set(&mut self, key: &str, value: Value);
}

#[derive(Clone, Debug, PartialEq)]
enum Token {
    Num(f64),
    Str(String),
    Ident(String),
    Op(&'static str),
}

const OPS: &[&str] = &[
    ":=", "+=", "-=", "*=", "/=", "==", "!=", "<=", ">=", "&&", "||", "=", "<", ">", "+", "-", "*",
    "/", "!", "(", ")", "?", ":", ";",
];

fn tokenize(src: &str) -> Result<Vec<Token>, String> {
    let chars: Vec<char> = src.chars().collect();
    let mut tokens = Vec::new();
    let mut i = 0;
    while i < chars.len() {
        let c = chars[i];
        if c.is_whitespace() {
            i += 1;
        } else if c.is_ascii_digit()
            || (c == '.' && chars.get(i + 1).is_some_and(char::is_ascii_digit))
        {
            let start = i;
            while i < chars.len() && (chars[i].is_ascii_digit() || chars[i] == '.') {
                i += 1;
            }
            let text: String = chars[start..i].iter().collect();
            tokens.push(Token::Num(
                text.parse().map_err(|_| format!("bad number `{text}`"))?,
            ));
        } else if c == '\'' || c == '"' {
            let start = i + 1;
            i += 1;
            while i < chars.len() && chars[i] != c {
                i += 1;
            }
            if i >= chars.len() {
                return Err("unterminated string".to_string());
            }
            tokens.push(Token::Str(chars[start..i].iter().collect()));
            i += 1;
        } else if c.is_alphabetic() || c == '_' || c == '@' {
            let start = i;
            while i < chars.len()
                && (chars[i].is_alphanumeric() || matches!(chars[i], '_' | '@' | '.'))
            {
                i += 1;
            }
            tokens.push(Token::Ident(chars[start..i].iter().collect()));
        } else {
            let rest: String = chars[i..chars.len().min(i + 2)].iter().collect();
            let op = OPS
                .iter()
                .find(|op| rest.starts_with(**op))
                .ok_or_else(|| format!("unexpected `{c}`"))?;
            tokens.push(Token::Op(op));
            i += op.chars().count();
        }
    }
    Ok(tokens)
}

struct Parser<'a, E: Env> {
    tokens: Vec<Token>,
    pos: usize,
    env: &'a mut E,
}

impl<E: Env> Parser<'_, E> {
    fn peek(&self) -> Option<&Token> {
        self.tokens.get(self.pos)
    }

    fn eat(&mut self, op: &str) -> bool {
        if matches!(self.peek(), Some(Token::Op(o)) if *o == op) {
            self.pos += 1;
            true
        } else {
            false
        }
    }

    fn statement(&mut self) -> Result<Value, String> {
        if let (Some(Token::Ident(name)), Some(Token::Op(op))) = (
            self.tokens.get(self.pos).cloned(),
            self.tokens.get(self.pos + 1).cloned(),
        ) && matches!(op, ":=" | "=" | "+=" | "-=" | "*=" | "/=")
        {
            self.pos += 2;
            let rhs = self.ternary()?;
            let value = match op {
                ":=" | "=" => rhs,
                _ => {
                    let current = self
                        .env
                        .get(&name)
                        .ok_or_else(|| format!("`{name}` is not on the blackboard"))?;
                    binary(&op[..1], &current, &rhs)?
                }
            };
            self.env.set(&name, value.clone());
            return Ok(value);
        }
        self.ternary()
    }

    fn ternary(&mut self) -> Result<Value, String> {
        let cond = self.or()?;
        if self.eat("?") {
            let a = self.ternary()?;
            if !self.eat(":") {
                return Err("expected `:`".to_string());
            }
            let b = self.ternary()?;
            return Ok(if truthy(&cond) { a } else { b });
        }
        Ok(cond)
    }

    fn or(&mut self) -> Result<Value, String> {
        let mut lhs = self.and()?;
        while self.eat("||") {
            let rhs = self.and()?;
            lhs = Value::Bool(truthy(&lhs) || truthy(&rhs));
        }
        Ok(lhs)
    }

    fn and(&mut self) -> Result<Value, String> {
        let mut lhs = self.comparison()?;
        while self.eat("&&") {
            let rhs = self.comparison()?;
            lhs = Value::Bool(truthy(&lhs) && truthy(&rhs));
        }
        Ok(lhs)
    }

    fn comparison(&mut self) -> Result<Value, String> {
        let mut lhs = self.sum()?;
        loop {
            let op = match self.peek() {
                Some(Token::Op(op @ ("==" | "!=" | "<" | "<=" | ">" | ">="))) => *op,
                _ => return Ok(lhs),
            };
            self.pos += 1;
            let rhs = self.sum()?;
            lhs = Value::Bool(compare(op, &lhs, &rhs)?);
        }
    }

    fn sum(&mut self) -> Result<Value, String> {
        let mut lhs = self.product()?;
        loop {
            let op = match self.peek() {
                Some(Token::Op(op @ ("+" | "-"))) => *op,
                _ => return Ok(lhs),
            };
            self.pos += 1;
            let rhs = self.product()?;
            lhs = binary(op, &lhs, &rhs)?;
        }
    }

    fn product(&mut self) -> Result<Value, String> {
        let mut lhs = self.unary()?;
        loop {
            let op = match self.peek() {
                Some(Token::Op(op @ ("*" | "/"))) => *op,
                _ => return Ok(lhs),
            };
            self.pos += 1;
            let rhs = self.unary()?;
            lhs = binary(op, &lhs, &rhs)?;
        }
    }

    fn unary(&mut self) -> Result<Value, String> {
        if self.eat("!") {
            return Ok(Value::Bool(!truthy(&self.unary()?)));
        }
        if self.eat("-") {
            let v = self.unary()?;
            return number(&v)
                .map(|n| json_num(-n))
                .ok_or_else(|| "`-` needs a number".to_string());
        }
        self.atom()
    }

    fn atom(&mut self) -> Result<Value, String> {
        let token = self
            .peek()
            .cloned()
            .ok_or_else(|| "unexpected end of script".to_string())?;
        self.pos += 1;
        match token {
            Token::Num(n) => Ok(json_num(n)),
            Token::Str(s) => Ok(Value::String(s)),
            Token::Ident(name) => match name.as_str() {
                "true" => Ok(Value::Bool(true)),
                "false" => Ok(Value::Bool(false)),
                _ => self
                    .env
                    .get(&name)
                    .ok_or_else(|| format!("`{name}` is not on the blackboard")),
            },
            Token::Op("(") => {
                let v = self.ternary()?;
                if !self.eat(")") {
                    return Err("expected `)`".to_string());
                }
                Ok(v)
            }
            Token::Op(op) => Err(format!("unexpected `{op}`")),
        }
    }
}

/// Run a script; the value of its last statement.
pub fn run(src: &str, env: &mut impl Env) -> Result<Value, String> {
    let tokens = tokenize(src)?;
    let mut parser = Parser {
        tokens,
        pos: 0,
        env,
    };
    let mut last = Value::Null;
    while parser.peek().is_some() {
        if parser.eat(";") {
            continue;
        }
        last = parser.statement()?;
        if parser.peek().is_some() && !parser.eat(";") {
            return Err(format!("unexpected {:?}", parser.peek().unwrap()));
        }
    }
    Ok(last)
}

/// Evaluate a condition script.
pub fn eval_bool(src: &str, env: &mut impl Env) -> Result<bool, String> {
    run(src, env).map(|v| truthy(&v))
}

pub fn truthy(v: &Value) -> bool {
    match v {
        Value::Null => false,
        Value::Bool(b) => *b,
        Value::Number(n) => n.as_f64() != Some(0.0),
        Value::String(s) => !matches!(s.trim(), "" | "0" | "false" | "False" | "FALSE"),
        Value::Array(a) => !a.is_empty(),
        Value::Object(o) => !o.is_empty(),
    }
}

/// A number, also from a numeric string (blackboard entries set from XML are strings).
pub fn number(v: &Value) -> Option<f64> {
    match v {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.trim().parse().ok(),
        Value::Bool(b) => Some(f64::from(u8::from(*b))),
        _ => None,
    }
}

pub fn json_num(n: f64) -> Value {
    if n.fract() == 0.0 && n.abs() < 9.0e15 {
        Value::from(n as i64)
    } else {
        serde_json::Number::from_f64(n).map_or(Value::Null, Value::Number)
    }
}

/// How a value reads as text, e.g. in a port chip.
pub fn display(v: &Value) -> String {
    match v {
        Value::String(s) => s.clone(),
        other => other.to_string(),
    }
}

fn binary(op: &str, a: &Value, b: &Value) -> Result<Value, String> {
    if op == "+" && (matches!(a, Value::String(_)) || matches!(b, Value::String(_))) {
        if let (Some(x), Some(y)) = (number(a), number(b)) {
            return Ok(json_num(x + y));
        }
        return Ok(Value::String(display(a) + &display(b)));
    }
    let (x, y) = (
        number(a).ok_or_else(|| format!("`{op}` needs numbers, got {a}"))?,
        number(b).ok_or_else(|| format!("`{op}` needs numbers, got {b}"))?,
    );
    Ok(json_num(match op {
        "+" => x + y,
        "-" => x - y,
        "*" => x * y,
        "/" => x / y,
        _ => unreachable!(),
    }))
}

fn compare(op: &str, a: &Value, b: &Value) -> Result<bool, String> {
    if let (Some(x), Some(y)) = (number(a), number(b)) {
        return Ok(match op {
            "==" => x == y,
            "!=" => x != y,
            "<" => x < y,
            "<=" => x <= y,
            ">" => x > y,
            _ => x >= y,
        });
    }
    let (x, y) = (display(a), display(b));
    Ok(match op {
        "==" => x == y,
        "!=" => x != y,
        "<" => x < y,
        "<=" => x <= y,
        ">" => x > y,
        _ => x >= y,
    })
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use super::*;

    #[derive(Default)]
    struct Map(HashMap<String, Value>);

    impl Env for Map {
        fn get(&self, key: &str) -> Option<Value> {
            self.0.get(key).cloned()
        }
        fn set(&mut self, key: &str, value: Value) {
            self.0.insert(key.to_string(), value);
        }
    }

    #[test]
    fn assignments_and_expressions() {
        let mut env = Map::default();
        run("a := 1; b = 'x'; a += 2; c := a * 2 + 1", &mut env).unwrap();
        assert_eq!(env.get("a"), Some(Value::from(3)));
        assert_eq!(env.get("b"), Some(Value::from("x")));
        assert_eq!(env.get("c"), Some(Value::from(7)));
        assert!(eval_bool("a == 3 && b == 'x' && !(c < 7)", &mut env).unwrap());
        assert!(eval_bool("a > 2 ? true : false", &mut env).unwrap());
        assert!(!eval_bool("a != 3 || b == 'y'", &mut env).unwrap());
        // Values set from XML are strings, but compare as numbers.
        env.set("n", Value::from("4"));
        assert!(eval_bool("n > 3", &mut env).unwrap());
        assert!(run("missing + 1", &mut env).is_err());
    }
}
