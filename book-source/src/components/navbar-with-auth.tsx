import React from 'react';
import { useAuth } from './auth/auth-provider';

const NavbarWithAuth: React.FC = () => {
  const { user, isAuthenticated, isLoading, logout } = useAuth();

  if (isLoading) {
    return (
      <div className="navbar__item">
        <span>Loading...</span>
      </div>
    );
  }

  return (
    <div className="navbar__item">
      {isAuthenticated ? (
        <div className="dropdown dropdown--right dropdown--align-bottom">
          <span className="navbar__link">
            {user?.name || user?.email} ▾
          </span>
          <ul className="dropdown__menu">
            <li>
              <a className="dropdown__link" href="/profile">
                Profile
              </a>
            </li>
            <li>
              <button
                className="dropdown__link"
                onClick={logout}
                style={{ width: '100%', textAlign: 'left', background: 'none', border: 'none', cursor: 'pointer' }}
              >
                Logout
              </button>
            </li>
          </ul>
        </div>
      ) : (
        <div className="navbar__link--auth-buttons">
          <a className="button button--secondary button--sm margin-horiz--sm" href="/login">
            Login
          </a>
          <a className="button button--primary button--sm" href="/signup">
            Sign Up
          </a>
        </div>
      )}
    </div>
  );
};

export default NavbarWithAuth;