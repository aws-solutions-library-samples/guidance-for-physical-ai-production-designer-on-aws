//! Authorization beyond "the workspace's owner": the roles of a caller ([`Roles`]), resolved by one
//! seam, [`Authz`]. Today one role, [`Roles::catalogue_admin`]: may write and organise the shared
//! catalogue. Every write is checked here on the backend; the viewer only shows what the roles allow.
//!
//! Who the caller is ([`Caller`]) comes from the request after admission (the edge's login,
//! `X-User-Alias`, or a loopback caller without the edge), never from the request body.
//!
//! **Today: config.** [`ConfigAuthz`], from `--catalogue-admins` / `FACTORY_OS_CATALOGUE_ADMINS`:
//! * `all`: every admitted caller; `none`: nobody; `alice,bob`: those aliases (a loopback caller
//!   without the edge counts as the workspace's owner, or `local` without one).
//! * unset: the local user only (a loopback caller without the edge: a pure local backend's user
//!   is the admin of its local catalogue); edge callers are not.
//!
//! **Later: the identity provider.** Replace [`ConfigAuthz`] with an [`Authz`] that reads group
//! membership (the edge's identity provider's groups, Cognito groups, an IdP claim the edge
//! forwards, verified) at the spot marked `IDENTITY-PROVIDER ROLES` below; nothing else changes.
//! The IAM side (the workspace role's `FactoryosCatalogueAdmin` tag,
//! infra/lib/workspace-provisioner.ts) must follow the same source, or S3 refuses what the backend
//! allows.

use std::collections::BTreeSet;
use std::net::SocketAddr;
use std::sync::Arc;

use axum::http::HeaderMap;
pub use factoryos_protocol::storage::Roles;

/// An admitted caller.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Caller {
    /// The edge's `X-User-Alias`; `None` for a loopback caller without the edge.
    pub alias: Option<String>,
}

impl Caller {
    /// From an admitted request's headers (see `services::files::FilesService::admit`).
    pub fn of(_peer: SocketAddr, headers: &HeaderMap) -> Self {
        let alias = headers.get("x-user-alias").and_then(|v| v.to_str().ok()).map(str::trim).filter(|a| !a.is_empty()).map(str::to_owned);
        Self { alias }
    }
}

/// Resolves a caller's roles. The one seam for authorization decisions beyond ownership.
pub trait Authz: Send + Sync {
    fn roles(&self, caller: &Caller) -> Roles;
}

/// Who is a catalogue admin, as configured.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Admins {
    All,
    None,
    Only(BTreeSet<String>),
}

impl std::str::FromStr for Admins {
    type Err = String;
    fn from_str(spec: &str) -> Result<Self, String> {
        match spec.trim() {
            "all" | "*" => Ok(Self::All),
            "none" | "" => Ok(Self::None),
            list => {
                let aliases: BTreeSet<String> = list.split(',').map(str::trim).filter(|a| !a.is_empty()).map(str::to_owned).collect();
                if let Some(bad) = aliases.iter().find(|a| !factoryos_protocol::workspace::is_valid_name(a) || *a == "all" || *a == "none") {
                    return Err(format!("catalogue admins: {bad:?} is not an alias (all, none, or alias,alias,…)"));
                }
                Ok(Self::Only(aliases))
            }
        }
    }
}

/// [`Authz`] from config (see the module docs).
pub struct ConfigAuthz {
    /// `None`: unset (the local user only).
    admins: Option<Admins>,
    /// `--workspace`: who a loopback caller without the edge is.
    owner: Option<String>,
}

impl ConfigAuthz {
    pub fn new(admins: Option<&str>, owner: Option<String>) -> anyhow::Result<Arc<Self>> {
        let admins = admins.map(str::parse::<Admins>).transpose().map_err(anyhow::Error::msg)?;
        Ok(Arc::new(Self { admins, owner }))
    }

    pub fn describe(&self) -> String {
        match &self.admins {
            None => "the local user only (FACTORY_OS_CATALOGUE_ADMINS unset)".to_owned(),
            Some(Admins::All) => "everyone".to_owned(),
            Some(Admins::None) => "nobody".to_owned(),
            Some(Admins::Only(list)) => list.iter().cloned().collect::<Vec<_>>().join(", "),
        }
    }
}

impl Authz for ConfigAuthz {
    fn roles(&self, caller: &Caller) -> Roles {
        // IDENTITY-PROVIDER ROLES: when the edge forwards verified group claims, decide here (or in
        // another `Authz`) from those instead of the configured list.
        let catalogue_admin = match (&self.admins, &caller.alias) {
            (None, alias) => alias.is_none(),
            (Some(Admins::All), _) => true,
            (Some(Admins::None), _) => false,
            (Some(Admins::Only(list)), Some(alias)) => list.contains(alias),
            (Some(Admins::Only(list)), None) => list.contains(self.owner.as_deref().unwrap_or("local")),
        };
        Roles { catalogue_admin }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roles(admins: Option<&str>, alias: Option<&str>) -> bool {
        let authz = ConfigAuthz::new(admins, Some("jdoe".into())).unwrap();
        authz.roles(&Caller { alias: alias.map(str::to_owned) }).catalogue_admin
    }

    #[test]
    fn parses_all_list_none() {
        assert_eq!("all".parse::<Admins>(), Ok(Admins::All));
        assert_eq!(" none ".parse::<Admins>(), Ok(Admins::None));
        assert_eq!("".parse::<Admins>(), Ok(Admins::None));
        assert_eq!("bob, alice,".parse::<Admins>(), Ok(Admins::Only(["alice".to_owned(), "bob".to_owned()].into())));
        assert!("al ice".parse::<Admins>().is_err());
        assert!("../x".parse::<Admins>().is_err());
        assert!(ConfigAuthz::new(Some("a,b c"), None).is_err());
    }

    #[test]
    fn resolves_roles() {
        // Unset: the local user (no edge) only.
        assert!(roles(None, None));
        assert!(!roles(None, Some("jdoe")));
        assert!(roles(Some("all"), Some("alice")) && roles(Some("all"), None));
        assert!(!roles(Some("none"), Some("jdoe")) && !roles(Some("none"), None));
        assert!(roles(Some("jdoe,bob"), Some("bob")) && !roles(Some("jdoe,bob"), Some("alice")));
        // Without the edge, the owner by name.
        assert!(roles(Some("jdoe"), None) && !roles(Some("bob"), None));
    }
}
